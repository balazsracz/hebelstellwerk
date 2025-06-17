/// Uses a Nucleo 303 or 091 with the OpenLCB IO Board extension to drive
/// servos.

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/Blinker.h"
#include "utils/GpioDebug.h"
#include "utils/GpioCopier.h"
#include "utils/ArduinoStm32DmaPwm.h"
#include "utils/ServoGpio.h"




#ifndef ARDUINO
#error baaa
#endif

int EXT_LAT = PC11;
int INT_LAT = PA6;



enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

  SERVOS = 100,
  SERVO1,
  SERVO2,
  SERVO3,
  SERVO4,
};

enum PwmPin : pwm_pin_t {
  PWM_DMA = 200,
  PWM_SERVO1 = PWM_DMA,
  PWM_SERVO2,
  PWM_SERVO3,
  PWM_SERVO4,
};

DmaPwm g_pwm(PWM_DMA, {PC9, PC7, PC8, PC6});

ServoGpio servos[] = {
  {SERVO1, PWM_SERVO1, 45, 135, 300},
  {SERVO2, PWM_SERVO2, 0, 180, 300},
  {SERVO3, PWM_SERVO3, 70, 110, 300},
  {SERVO4, PWM_SERVO4, 80, 100, 300},
};

std::initializer_list<GpioCopyInstance> g_shadows{
  {ONBOARD_BTN, SERVO1, true},
};

GpioCopy g_gpio_shadow{g_shadows};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);
  
  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");
  delay(10);

  Executor::instance()->begin();
  Serial.println("begin complete");
  delay(10);
}

void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
