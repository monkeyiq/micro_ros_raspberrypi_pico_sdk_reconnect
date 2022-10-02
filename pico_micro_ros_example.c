
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTING,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

const uint LED_PIN = 25;

const uint PIN_RED   = 10;
const uint PIN_GREEN = 11;
const uint PIN_BLUE  = 12;
const uint PIN_BLUE2  = 14;
const uint PIN_BLUE3  = 15;

bool ledState = 1;
int ping_failures = 0;


rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
bool micro_ros_init_successful;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

bool create_entities()
{
    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    return true;
}


void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
  rcl_publisher_fini(&publisher,  &node);
  gpio_put(PIN_BLUE2, 0 );
  rcl_timer_fini(&timer);
//  rcl_subscription_fini(&sub_d0, &node);

  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  gpio_put(PIN_BLUE3, 0 );
  rclc_support_fini(&support);  

}


void loop()
{
    switch (state) {
        case WAITING_AGENT:
            gpio_put(PIN_RED, 0 ); 
            EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            gpio_put(PIN_RED, 1 );            
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTING : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            }
            break;
        case AGENT_CONNECTING:
            gpio_put(PIN_GREEN, 0 );
            EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            gpio_put(PIN_GREEN, 1 );
            break;
        case AGENT_CONNECTED:
            gpio_put(PIN_GREEN, 0 );
            EXECUTE_EVERY_N_MS(100, state = (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            gpio_put(PIN_GREEN, 1 );
            break;
        case AGENT_DISCONNECTED:
            gpio_put(PIN_BLUE, 0 );
            destroy_entities();
            state = WAITING_AGENT;
            gpio_put(PIN_BLUE, 1 );
            gpio_put(PIN_BLUE2, 1 );
            gpio_put(PIN_BLUE3, 1 );
            break;
        default:
            break;
    }
    
    ledState = !ledState;
    gpio_put(LED_PIN, ledState );
}

int main()
{
    pico_serial_init();
    msg.data = 0;

    ledState = !ledState;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, ledState );

    gpio_init(PIN_RED);
    gpio_set_dir(PIN_RED, GPIO_OUT);
    gpio_put(PIN_RED, 1 );
    gpio_init(PIN_GREEN);
    gpio_set_dir(PIN_GREEN, GPIO_OUT);
    gpio_put(PIN_GREEN, 1 );
    gpio_init(PIN_BLUE);
    gpio_set_dir(PIN_BLUE, GPIO_OUT);
    gpio_put(PIN_BLUE, 1 );

    gpio_init(PIN_BLUE2);
    gpio_set_dir(PIN_BLUE2, GPIO_OUT);
    gpio_put(PIN_BLUE2, 1 );
    gpio_init(PIN_BLUE3);
    gpio_set_dir(PIN_BLUE3, GPIO_OUT);
    gpio_put(PIN_BLUE3, 1 );
        
    state = WAITING_AGENT;

    while( true ) {
        loop();
    }
}
