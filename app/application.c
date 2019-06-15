#include <application.h>
#include <bc_servo.h>

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

bc_servo_t servo_p1;

bc_tmp112_t tmp112;

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

bc_tick_t safety_timeout = 0;

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_HOLD)
    {
        bc_led_pulse(&led, 2000);

        if (state == CLOSED)
        {
            open_timer = 4000;
        }
    }

    if (event == BC_BUTTON_EVENT_CLICK)
    {
        bc_led_pulse(&led, 200);

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

void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *param)
{
    (void) param;

    if (event == BC_TMP112_EVENT_UPDATE)
    {
        float temperature;
        bc_tmp112_get_temperature_celsius(&tmp112, &temperature);
        bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);
        bc_log_debug("Temperature: %0.2f °C", temperature);
    }
}

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_pulse(&led, 2000);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);
    bc_button_set_hold_time(&button, 1000);

    // Init radio
    bc_radio_init(BC_RADIO_MODE_NODE_LISTENING);

    bc_servo_init(&servo_p1, BC_PWM_P1);

    bc_pwm_disable(BC_PWM_P1);

    // Initialize closed end-stop switch
    bc_gpio_init(BC_GPIO_P8);
    bc_gpio_set_mode(BC_GPIO_P8, BC_GPIO_MODE_INPUT);
    bc_gpio_set_pull(BC_GPIO_P8, BC_GPIO_PULL_UP);

    // Onboard thermometer
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, BC_TAG_TEMPERATURE_I2C_ADDRESS_ALTERNATE);
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_INTERVAL);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);

    bc_radio_pairing_request("window-servo", VERSION);

}

// Use LED bightness topic to receive value 0-255 that means 0 - 25.5 second motor rotation
void bc_radio_node_on_led_strip_brightness_set(uint64_t *id, uint8_t *brightness)
{
    (void) id;

    // Convert tenths of seconds to the milleconds
    open_timer = *brightness * 100;

    bc_log_debug("open_timer: %d ms", open_timer);
}

bool window_is_closed()
{
    return bc_gpio_get_input(BC_GPIO_P8) == 0;
}

void motor_open()
{
    bc_pwm_enable(BC_PWM_P1);
    bc_servo_set_angle(&servo_p1, SERVO_OPENING);
    bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);
}

void motor_close()
{
    bc_pwm_enable(BC_PWM_P1);
    bc_servo_set_angle(&servo_p1, SERVO_CLOSING);
    bc_led_set_mode(&led, BC_LED_MODE_BLINK);
}

void motor_stop()
{
    bc_pwm_disable(BC_PWM_P1);
    bc_led_set_mode(&led, BC_LED_MODE_OFF);
}

void application_task(void)
{


    switch (state)
    {

        case CLOSE:
        {
            motor_close();
            state = CLOSING;
            bc_log_debug("CLOSE");
            // Safety closing timer
            safety_timeout = bc_tick_get() + SAFETY_CLOSING_TIMEOUT;
            break;
        }

        case CLOSING:
        {
            if (window_is_closed())
            {
                motor_stop();
                bc_log_debug("Closed");
                bc_radio_pub_int("servo/-/closed", NULL);
                state = CLOSED;
            }

            if (bc_tick_get() > safety_timeout)
            {
                motor_stop();
                state = CLOSED;
                bc_log_debug("Safety STOP");
                bc_radio_pub_int("servo/-/safety-stop", NULL);
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
            bc_log_debug("OPEN");

            // Plan task after opening is complete
            bc_scheduler_plan_current_relative(open_timer);
            return;
            break;
        }

        case OPENING:
        {
            bc_radio_pub_int("servo/-/opened", &open_timer);
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
                bc_log_debug("Close");
            }
            break;
        }

        default:
        {
            break;
        }
    }

    //bc_servo_set_angle(&servo_p1, smooth_get(&sm_p1));
    bc_scheduler_plan_current_relative(20);
}
