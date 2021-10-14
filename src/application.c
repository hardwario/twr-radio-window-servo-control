#include <application.h>
#include <twr_servo.h>

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

twr_servo_t servo_p1;

twr_tmp112_t tmp112;

// Define servo angle speeds for opening, idle, closing
#define SERVO_MIDDLE_POSITION 93
#define SERVO_OPENING 110
#define SERVO_CLOSING 70

#define SAFETY_CLOSING_TIMEOUT 5000

// Send temperature every 10 minutes
#define TEMPERATURE_INTERVAL (10 * 60 * 1000)

typedef enum
{
    CLOSE,
    CLOSING,
    CLOSED,
    OPEN,
    OPENING,
    OPENED
} state_t;

state_t state = CLOSE;

int open_timer = 0;

twr_tick_t safety_timeout = 0;

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    if (event == TWR_BUTTON_EVENT_HOLD)
    {
        twr_led_pulse(&led, 2000);

        if (state == CLOSED)
        {
            open_timer = 4000;
        }
    }

    if (event == TWR_BUTTON_EVENT_CLICK)
    {
        twr_led_pulse(&led, 200);

        if (state == CLOSED)
        {
            open_timer = 2000;
        }

        if (state == OPENED)
        {
            open_timer = 0;
        }
    }
}

void tmp112_event_handler(twr_tmp112_t *self, twr_tmp112_event_t event, void *param)
{
    (void) param;

    if (event == TWR_TMP112_EVENT_UPDATE)
    {
        float temperature;
        twr_tmp112_get_temperature_celsius(&tmp112, &temperature);
        twr_radio_pub_temperature(TWR_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);
        twr_log_debug("Temperature: %0.2f °C", temperature);
    }
}

void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_pulse(&led, 2000);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_event_handler(&button, button_event_handler, NULL);
    twr_button_set_hold_time(&button, 1000);

    // Init radio
    twr_radio_init(TWR_RADIO_MODE_NODE_LISTENING);

    twr_servo_init(&servo_p1, TWR_PWM_P1);

    twr_pwm_disable(TWR_PWM_P1);

    // Initialize closed end-stop switch
    twr_gpio_init(TWR_GPIO_P8);
    twr_gpio_set_mode(TWR_GPIO_P8, TWR_GPIO_MODE_INPUT);
    twr_gpio_set_pull(TWR_GPIO_P8, TWR_GPIO_PULL_UP);

    // Onboard thermometer
    twr_tmp112_init(&tmp112, TWR_I2C_I2C0, TWR_TAG_TEMPERATURE_I2C_ADDRESS_ALTERNATE);
    twr_tmp112_set_update_interval(&tmp112, TEMPERATURE_INTERVAL);
    twr_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);

    twr_radio_pairing_request("window-servo", VERSION);

}

// Use LED bightness topic to receive value 0-255 that means 0 - 25.5 second motor rotation
void twr_radio_node_on_led_strip_brightness_set(uint64_t *id, uint8_t *brightness)
{
    (void) id;

    // Convert tenths of seconds to the milleconds
    open_timer = *brightness * 100;

    twr_log_debug("open_timer: %d ms", open_timer);
}

bool window_is_closed()
{
    return twr_gpio_get_input(TWR_GPIO_P8) == 0;
}

void motor_open()
{
    twr_pwm_enable(TWR_PWM_P1);
    twr_servo_set_angle(&servo_p1, SERVO_OPENING);
    twr_led_set_mode(&led, TWR_LED_MODE_BLINK_FAST);
}

void motor_close()
{
    twr_pwm_enable(TWR_PWM_P1);
    twr_servo_set_angle(&servo_p1, SERVO_CLOSING);
    twr_led_set_mode(&led, TWR_LED_MODE_BLINK);
}

void motor_stop()
{
    twr_pwm_disable(TWR_PWM_P1);
    twr_led_set_mode(&led, TWR_LED_MODE_OFF);
}

void application_task(void)
{


    switch (state)
    {

        case CLOSE:
        {
            motor_close();
            state = CLOSING;
            twr_log_debug("CLOSE");
            // Safety closing timer
            safety_timeout = twr_tick_get() + SAFETY_CLOSING_TIMEOUT;
            break;
        }

        case CLOSING:
        {
            if (window_is_closed())
            {
                motor_stop();
                twr_log_debug("Closed");
                twr_radio_pub_int("servo/-/closed", NULL);
                state = CLOSED;
            }

            if (twr_tick_get() > safety_timeout)
            {
                motor_stop();
                state = CLOSED;
                twr_log_debug("Safety STOP");
                twr_radio_pub_int("servo/-/safety-stop", NULL);
            }
            break;
        }

        case CLOSED:
        {
            if (open_timer)
            {
                state = OPEN;
            }
            break;
        }

        case OPEN:
        {
            state = OPENING;
            motor_open();
            twr_log_debug("OPEN");

            // Plan task after opening is complete
            twr_scheduler_plan_current_relative(open_timer);
            return;
            break;
        }

        case OPENING:
        {
            twr_radio_pub_int("servo/-/opened", &open_timer);
            state = OPENED;
            break;
        }

        case OPENED:
        {
            motor_stop();

            // If value is zero - close window
            if (open_timer == 0)
            {
                state = CLOSE;
                twr_log_debug("Close");
            }
            break;
        }

        default:
        {
            break;
        }
    }

    //twr_servo_set_angle(&servo_p1, smooth_get(&sm_p1));
    twr_scheduler_plan_current_relative(20);
}
