#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

// Nodo y soporte
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// Publicador
rcl_publisher_t publisher;
std_msgs__msg__String msg_out; // Mensaje de salida
rclc_executor_t executor_pub;
rcl_timer_t timer;

// Suscriptor
rcl_subscription_t subscriber;
std_msgs__msg__String msg_in; // Mensaje recibido
rclc_executor_t executor_sub;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    delay(100);
  }
}

// Callback del temporizador
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Publica un mensaje fijo en el tópico de salida
    const char *heartbeat_message = "ESP32: Enviando latido...";
    snprintf(msg_out.data.data, msg_out.data.capacity, "%s", heartbeat_message);
    msg_out.data.size = strlen(heartbeat_message);

    RCSOFTCHECK(rcl_publish(&publisher, &msg_out, NULL));
    Serial.println("Mensaje publicado: " + String(heartbeat_message));
  }
}


void subscription_callback(const void *msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;

  // Guardar el mensaje recibido en una variable global
  snprintf(msg_in.data.data, msg_in.data.capacity, "%s", msg->data.data);
  msg_in.data.size = strlen(msg_in.data.data);

  // Imprimir el valor recibido
  Serial.print("Mensaje recibido: ");
  Serial.println(msg_in.data.data);
}

void setup()
{
  // Configurar transporte micro-ROS
  set_microros_transports();

  // Inicializar serial para depuración
  Serial.begin(115200);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Crear soporte
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crear nodo
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Crear suscriptor
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "esp32_in"));

  // Crear publicador
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "esp32_out"));

  // Crear temporizador, llamado cada 1000 ms para publicar
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // Crear ejecutores
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg_in, &subscription_callback, ON_NEW_DATA));

  // Inicializar mensaje de salida
  msg_out.data.data = (char *)malloc(100 * sizeof(char));
  msg_out.data.capacity = 100;
  msg_out.data.size = 0;

  // Inicializar mensaje de entrada
  msg_in.data.data = (char *)malloc(100 * sizeof(char));
  msg_in.data.capacity = 100;
  msg_in.data.size = 0;
}

void loop()
{
  // Ejecutar los ejecutores
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
