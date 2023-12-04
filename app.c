#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <driver/gpio.h>

#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn)                                                                \
  {                                                                                \
    rcl_ret_t temp_rc = fn;                                                        \
    if ((temp_rc != RCL_RET_OK))                                                   \
    {                                                                              \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
    }                                                                              \
  }
#define RCSOFTCHECK(fn)                                                              \
  {                                                                                  \
    rcl_ret_t temp_rc = fn;                                                          \
    if ((temp_rc != RCL_RET_OK))                                                     \
    {                                                                                \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                                \
  }

/*--------------------------
 * Switch State
 *--------------------------*/

// Please connect 220V AC to NC (Normally Closed) and COM (Common) of relay.
#define RELAY_GPIO GPIO_NUM_26
#define LED_BUILTIN GPIO_NUM_2

// DO NOT USE gpio_get_level(RELAY_GPIO)!
// gpio_get_level() always returns 0 for pins of GPIO_MODE_OUTPUT.
// See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv414gpio_get_level10gpio_num_t
bool b_switch_is_on = false;

static void configure_gpio(void)
{
  gpio_reset_pin(LED_BUILTIN);
  gpio_reset_pin(RELAY_GPIO);
  // Set the GPIO as a push/pull output
  gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
}

void set_switch(bool new_state)
{
  esp_err_t err = gpio_set_level(RELAY_GPIO, new_state ? 1 : 0);
  if (err == ESP_OK)
  {
    b_switch_is_on = new_state;
  }
  else
  {
    printf("Failed to set switch state: %d\n", err);
  }
}

// DO NOT USE usleep() in timer callback!
// See: https://micro.ros.org/docs/concepts/client_library/execution_management/
int64_t _next_toggle_msec = 0;
uint32_t _prev_led_level = 0;
// The `last_call_time` is nanoseconds. (1ms = 1000000ns)
// See: https://github.com/ros2/rcl/blob/iron/rcl/include/rcl/timer.h#L62-L64
void blink_led_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  int64_t now = rmw_uros_epoch_millis();
  if (now > _next_toggle_msec)
  {
    const int64_t led_interval_msec = 2000;
    const int64_t led_on = (b_switch_is_on ? 1900 : 100);
    if (timer != NULL)
    {
      if (_prev_led_level == 0)
      {
        gpio_set_level(LED_BUILTIN, 1);
        _prev_led_level = 1;
        _next_toggle_msec = now + led_on;
      }
      else
      {
        gpio_set_level(LED_BUILTIN, 0);
        _prev_led_level = 0;
        _next_toggle_msec = now + (led_interval_msec - led_on);
      }
    }
  }
}

void blink_led_while_waiting_agent(const unsigned int blink_us)
{
  // If agent is not connected, blink LED
  gpio_set_level(LED_BUILTIN, 1);
  usleep(blink_us);
  gpio_set_level(LED_BUILTIN, 0);
  usleep(blink_us);
}

/*--------------------------
 * ROS Publisher
 *--------------------------*/
rcl_publisher_t publisher;
std_msgs__msg__Bool msg;

void state_publisher_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    msg.data = b_switch_is_on;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

/*--------------------------
 * ROS Service
 *--------------------------*/
// char msg_buffer[32] = {"\n"};
void service_callback(const void *req, void *res)
{
  std_srvs__srv__SetBool_Request *req_in = (std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response *res_in = (std_srvs__srv__SetBool_Response *)res;

  // Request
  printf("Service request: %d (Current state: %d)\n", req_in->data, b_switch_is_on);
  set_switch(req_in->data);

  // Response
  res_in->success = (b_switch_is_on == req_in->data);
  /*
   * NOTE: `message.data` occurs "microROS service hangs on first call" issue.
   * See: https://answers.ros.org/question/410649/microros-service-hangs-on-first-call/
   * From the second call, it works well. However, the first call never returns.
   * This may be a bug of micro-ROS.
   */
  // sprintf(msg_buffer, "Switch is now %s.\n", g_switch_is_on ? "ON" : "OFF");
  // res_in->message.data = msg_buffer;
  // res_in->message.data = (g_switch_is_on ? "Switch is now ON." : "Switch is now OFF.");
}

/*--------------------------
 * Auto-connect to micro-ROS Agent
 * See 1: https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
 * See 2: https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/handle_reconnections/handle_reconnections.html
 *--------------------------*/
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_timer_t state_publisher_timer;
rcl_timer_t blink_led_timer;

rcl_service_t service;
std_srvs__srv__SetBool_Request req;
std_srvs__srv__SetBool_Response res;

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "skku_esp32_switch", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/skku_esp32_switch/switch_state"));

  // create timer
  const unsigned int state_publisher_interval_ms = 250;  // 250ms
  RCCHECK(rclc_timer_init_default(
    &state_publisher_timer,
    &support,
    RCL_MS_TO_NS(state_publisher_interval_ms),
    state_publisher_callback));

  const unsigned int led_timer_timeout = 100;  // 0.1sec
  RCCHECK(rclc_timer_init_default(
    &blink_led_timer,
    &support,
    RCL_MS_TO_NS(led_timer_timeout),
    blink_led_timer_callback));

  // create service
  RCCHECK(rclc_service_init_default(
    &service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "/skku_esp32_switch/set_switch"));

  // create executor
  // num_of_handles = the number of rclc_executor_add_*() calls
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &state_publisher_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &blink_led_timer));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

  return true;
}

void destroy_entities()
{
  // free resources
  RCCHECK(rcl_service_fini(&service, &node));
  RCCHECK(rcl_publisher_fini(&publisher, &node))
  RCCHECK(rcl_timer_fini(&state_publisher_timer));
  RCCHECK(rcl_timer_fini(&blink_led_timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
}

/*--------------------------
 * Main
 *--------------------------*/
void appMain(void *arg)
{
  configure_gpio();

  // main loop
  state = WAITING_AGENT;
  while (1)
  {
    switch (state)
    {
    case WAITING_AGENT:
      state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
      blink_led_while_waiting_agent(250000);  // microsecond (1000us = 1ms)
      break;

    case AGENT_AVAILABLE:
      state = (create_entities() == true) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
      }
      break;

    case AGENT_DISCONNECTED:
      gpio_set_level(LED_BUILTIN, 1);
      destroy_entities();
      gpio_set_level(LED_BUILTIN, 0);
      state = WAITING_AGENT;
      // esp_restart();
      break;

    default:
      break;
    }
  }
  // rclc_executor_spin(&executor);
}
