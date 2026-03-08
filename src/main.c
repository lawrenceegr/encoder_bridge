#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/input/input.h>
#include <string.h>

#define TRIGGER_MS      500
#define IDLE_TIMEOUT_S  5
#define NUM_ENCODERS    3
#define CMD_BUF_SIZE    24

static const struct gpio_dt_spec ccw_gpios[NUM_ENCODERS] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(ccw_1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(ccw_2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(ccw_3), gpios),
};

static const struct gpio_dt_spec cw_gpios[NUM_ENCODERS] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(cw_1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(cw_2), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(cw_3), gpios),
};

#define TRIGGER_NODE DT_NODELABEL(gpioa)
static const struct gpio_dt_spec trigger_gpio = {
	.port = DEVICE_DT_GET(TRIGGER_NODE),
	.pin = 11,
	.dt_flags = GPIO_ACTIVE_HIGH,
};

static const struct device *const qdec_devs[NUM_ENCODERS] = {
	DEVICE_DT_GET(DT_ALIAS(encoder_0)),
	DEVICE_DT_GET(DT_ALIAS(encoder_1)),
	DEVICE_DT_GET(DT_ALIAS(encoder_2)),
};

static uint32_t cw_counts[NUM_ENCODERS];
static uint32_t ccw_counts[NUM_ENCODERS];
static uint32_t last_event_time;
static volatile bool sleeping;

/*
 * Pending encoder event written by the ISR (input_cb) and consumed by
 * encoder_work_handler.  Mirrors the ZMK ec11 pattern where the ISR only
 * captures state and posts work; all GPIO configuration happens in the
 * workqueue context where gpio_pin_configure_dt() is safe to call.
 */
static struct {
	int  idx;
	int8_t dir; /* +1 = CW, -1 = CCW */
} pending_enc;

/* ZMK-style: ISR posts, workqueue processes (cf. ec11_work_cb / ec11_thread_cb) */
static struct k_work encoder_work;
static struct k_work_delayable trigger_off_work;
static struct k_timer idle_timer;
static struct gpio_callback trigger_cb_data;
static struct k_sem idle_sem;

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_pos;

static void clear_direction_outputs(void)
{
	for (int i = 0; i < NUM_ENCODERS; i++) {
		gpio_pin_set_dt(&cw_gpios[i], 0);
		gpio_pin_set_dt(&ccw_gpios[i], 0);
	}
}

/*
 * Runs in workqueue context (delayable work) — safe to reconfigure the trigger
 * pin and re-arm the GPIO interrupt.
 */
static void trigger_off_handler(struct k_work *work)
{
	clear_direction_outputs();
	gpio_pin_configure_dt(&trigger_gpio, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set_dt(&trigger_gpio, 0);
	gpio_pin_configure_dt(&trigger_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&trigger_gpio, GPIO_INT_EDGE_RISING);
}

static void set_direction(const struct gpio_dt_spec *active,
			  const struct gpio_dt_spec *inactive)
{
	gpio_pin_set_dt(inactive, 0);
	gpio_pin_set_dt(active, 1);
}

/*
 * encoder_work_handler — ZMK equivalent of ec11_thread_cb / ec11_work_cb.
 *
 * All GPIO configuration (gpio_pin_configure_dt) must happen here rather than
 * in the ISR, because gpio_pin_configure_dt is not guaranteed to be ISR-safe
 * on STM32 (it manipulates MODER/OTYPER/PUPDR registers).
 *
 * Pattern mirrored from ZMK ec11_trigger.c:
 *   ISR  → setup_int(false) + k_work_submit(&drv_data->work)
 *   work → handler(dev, trigger) + setup_int(true)
 */
static void encoder_work_handler(struct k_work *work)
{
	int idx = pending_enc.idx;
	int8_t dir = pending_enc.dir;

	/* Set direction outputs (gpio_pin_set_dt is ISR-safe, but doing it
	 * here keeps all GPIO operations together and avoids tearing between
	 * direction and trigger when events arrive back-to-back). */
	if (dir > 0) {
		set_direction(&cw_gpios[idx], &ccw_gpios[idx]);
	} else {
		set_direction(&ccw_gpios[idx], &cw_gpios[idx]);
	}

	/* Assert trigger.  gpio_pin_configure_dt is safe here (workqueue). */
	gpio_pin_configure_dt(&trigger_gpio, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set_dt(&trigger_gpio, 1);
	k_work_reschedule(&trigger_off_work, K_MSEC(TRIGGER_MS));
}

static int encoder_index(const struct device *dev)
{
	for (int i = 0; i < NUM_ENCODERS; i++) {
		if (dev == qdec_devs[i]) {
			return i;
		}
	}
	return -1;
}

/*
 * input_cb — ZMK equivalent of ec11_a_gpio_callback / ec11_b_gpio_callback.
 *
 * With CONFIG_INPUT_MODE_SYNCHRONOUS=y this runs in ISR context, so it must
 * stay minimal: record the event, update counters/timers, then post work.
 * No gpio_pin_configure_dt calls here.
 */
static void input_cb(struct input_event *evt, void *user_data)
{
	if (evt->type != INPUT_EV_REL) {
		return;
	}

	int idx = encoder_index(evt->dev);
	if (idx < 0) {
		return;
	}

	last_event_time = k_uptime_get_32();
	k_timer_start(&idle_timer, K_SECONDS(IDLE_TIMEOUT_S), K_NO_WAIT);

	if (evt->value > 0) {
		cw_counts[idx]++;
		printk("Enc %d: CW t=%u\n", idx, last_event_time);
		pending_enc.idx = idx;
		pending_enc.dir = 1;
	} else if (evt->value < 0) {
		ccw_counts[idx]++;
		printk("Enc %d: CCW t=%u\n", idx, last_event_time);
		pending_enc.idx = idx;
		pending_enc.dir = -1;
	} else {
		return;
	}

	/* Like ZMK's k_work_submit(&drv_data->work) in ec11_a_gpio_callback */
	k_work_submit(&encoder_work);
}

INPUT_CALLBACK_DEFINE(NULL, input_cb, NULL);

static void idle_timer_expiry(struct k_timer *timer)
{
	printk("Idle, sleep t=%u\n", k_uptime_get_32());
	sleeping = true;
	k_sem_give(&idle_sem);
}

/*
 * trigger_isr — called when the external device drives the trigger pin HIGH
 * (used as a wake signal).  This is a real ISR so it must only perform
 * ISR-safe operations: read flags, start timers, give semaphores.
 *
 * Do NOT call gpio_pin_configure_dt from here.  The original code called
 * signal_trigger() which reconfigured the pin as an output — unsafe in ISR
 * and would also fight the external device driving the same wire.
 */
static void trigger_isr(const struct device *dev, struct gpio_callback *cb,
			 uint32_t pins)
{
	if (sleeping) {
		sleeping = false;
		printk("Wake t=%u\n", k_uptime_get_32());
	}
	k_timer_start(&idle_timer, K_SECONDS(IDLE_TIMEOUT_S), K_NO_WAIT);
}

static void process_cmd(const char *cmd)
{
	if (strcmp(cmd, "enc stat") == 0) {
		for (int i = 0; i < NUM_ENCODERS; i++) {
			printk("Enc %d: CW=%u CCW=%u\n", i,
			       cw_counts[i], ccw_counts[i]);
		}
	} else if (strcmp(cmd, "enc reset") == 0) {
		memset(cw_counts, 0, sizeof(cw_counts));
		memset(ccw_counts, 0, sizeof(ccw_counts));
		printk("Counters reset\n");
	} else if (strcmp(cmd, "pwr sleep") == 0) {
		printk("Forcing sleep...\n");
		sleeping = true;
		k_sem_give(&idle_sem);
	} else if (strcmp(cmd, "pwr status") == 0) {
		uint32_t now = k_uptime_get_32();
		uint32_t idle = last_event_time ? (now - last_event_time) : now;
		printk("Up: %u ms\nLast: %u ms\nIdle: %u ms\n",
		       now, last_event_time, idle);
	} else if (cmd[0] != '\0') {
		printk("? %s\n", cmd);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) {
		return;
	}

	while (uart_fifo_read(dev, &c, 1) == 1) {
		if (c == '\r' || c == '\n') {
			if (cmd_pos > 0) {
				cmd_buf[cmd_pos] = '\0';
				process_cmd(cmd_buf);
				cmd_pos = 0;
			}
		} else if (cmd_pos < CMD_BUF_SIZE - 1) {
			cmd_buf[cmd_pos++] = c;
		}
	}
}

static int init_gpios(void)
{
	int ret;

	for (int i = 0; i < NUM_ENCODERS; i++) {
		if (!gpio_is_ready_dt(&ccw_gpios[i]) ||
		    !gpio_is_ready_dt(&cw_gpios[i])) {
			printk("GPIO %d not ready\n", i);
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&ccw_gpios[i], GPIO_OUTPUT_INACTIVE);
		if (ret) {
			printk("CCW %d configure failed: %d\n", i, ret);
			return ret;
		}
		ret = gpio_pin_configure_dt(&cw_gpios[i], GPIO_OUTPUT_INACTIVE);
		if (ret) {
			printk("CW %d configure failed: %d\n", i, ret);
			return ret;
		}
	}

	if (!gpio_is_ready_dt(&trigger_gpio)) {
		return -ENODEV;
	}
	gpio_pin_configure_dt(&trigger_gpio, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&trigger_gpio, GPIO_INT_EDGE_RISING);
	gpio_init_callback(&trigger_cb_data, trigger_isr, BIT(trigger_gpio.pin));
	gpio_add_callback(trigger_gpio.port, &trigger_cb_data);

	return 0;
}

int main(void)
{
	printk("Encoder Bridge starting\n");

	k_sem_init(&idle_sem, 0, 1);

	if (init_gpios()) {
		printk("GPIO init failed\n");
		return -1;
	}

	/* ZMK-style work init (cf. k_work_init(&drv_data->work, ec11_work_cb)) */
	k_work_init(&encoder_work, encoder_work_handler);
	k_work_init_delayable(&trigger_off_work, trigger_off_handler);
	k_timer_init(&idle_timer, idle_timer_expiry, NULL);
	k_timer_start(&idle_timer, K_SECONDS(IDLE_TIMEOUT_S), K_NO_WAIT);

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	printk("Encoder Bridge ready\n");

	while (1) {
		k_sem_take(&idle_sem, K_FOREVER);
		if (sleeping) {
			printk("Entering sleep t=%u\n", k_uptime_get_32());
			k_sleep(K_FOREVER);
			sleeping = false;
			printk("Woke t=%u\n", k_uptime_get_32());
			k_timer_start(&idle_timer, K_SECONDS(IDLE_TIMEOUT_S), K_NO_WAIT);
		}
	}

	return 0;
}
