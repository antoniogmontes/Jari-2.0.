#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <Adafruit_NeoPixel.h> // Biblioteca que permite controlar los colores de la tira LED


// -------------------------- VARIABLES ROS -----------------------------
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


// -------------------------- VARIABLES LEDS ----------------------------
//#define LED_PIN 13
#define PIN_LED 4 // Pin del bus de datos (GPIO D6 -> GPIO D9)
#define NLED 5  // Numero de leds que se encienden de la tira

Adafruit_NeoPixel tira = Adafruit_NeoPixel(NLED, PIN_LED, NEO_GRB + NEO_KHZ800);

// Codigos RGB tira LED
uint32_t vhappy = tira.Color(255, 165, 0);  // Orange '2'
uint32_t happy = tira.Color(255, 165, 0);   // Yellow '3'
uint32_t vsad = tira.Color(0, 0, 139);  // Blue dark '4'
uint32_t sad = tira.Color(0, 0, 255); // Blue '5'
uint32_t disgusted = tira.Color(0, 255, 0); // Green '6'
uint32_t angry = tira.Color(255, 0, 0); // Red '7'
uint32_t scared = tira.Color(255, 0, 255);  // Purple '8'
uint32_t serious = tira.Color(0, 0, 0); // Off '1'
uint32_t surprised = tira.Color(127, 255, 212); // Aquamarine '9'
uint32_t off = tira.Color(0, 0, 0); // Default Off '0'

// Macros 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//

void error_loop(){
  while(1){
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);

  decoder_command(msg);
  
}

void setup() {
  delay(5000);
  
  set_microros_transports();
  
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);  

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "send_command_arduino"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Inicializar el segundo puerto serial
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // Limpiar el buffer del puerto serial
  while (Serial2.available() > 0) {
      Serial2.read();
  }

  // Configuracion tira LED
  tira.begin();
  tira.show();
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void decoder_command(const std_msgs__msg__Int32 * msg) {
  for(int i = 0; i < NLED; i++) {
    switch(msg->data) {
      case 1:
        tira.setPixelColor(i, serious);
        break;
      case 2:
        tira.setPixelColor(i, vhappy);
        break;
      case 3:
        tira.setPixelColor(i, happy);
        break;
      case 4:
        tira.setPixelColor(i, vsad);
        break;
      case 5:
        tira.setPixelColor(i, sad);
        break;
      case 6:
        tira.setPixelColor(i, disgusted);
        break;
      case 7:
        tira.setPixelColor(i, angry);
        break;
      case 8:
        tira.setPixelColor(i, scared);
        break;
      case 9:
        tira.setPixelColor(i, surprised);
        break;
      default:
        tira.setPixelColor(i, off);
        break;
    }
    tira.show(); 
  }
}
