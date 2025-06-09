/// Uses a Nucleo 303 with the OpenLCB IO Board extension to drive external SPI connected GPIOs.

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include <SPI.h>
#include "utils/GpioSpi.h"


#ifndef ARDUINO
#error baaa
#endif

int EXT_LAT = PC11;

SPIClass g_ext_spi{PB5, PB4, PB3};

enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

  EXT_SPI = 100,

  EXT_2 = EXT_SPI + 8,
  EXT_OUT0 = EXT_2,
  EXT_OUT1,
  EXT_OUT2,
  EXT_OUT3,
  EXT_OUT4,
  EXT_OUT5,
  EXT_OUT6,
  EXT_OUT7,

  EXT_IN = EXT_2 + 8,
  EXT_IN0 = EXT_IN,
  EXT_IN1,
  EXT_IN2,
  EXT_IN3,
  EXT_IN4,
  EXT_IN5,
  EXT_IN6,
  EXT_IN7,

};

GpioSpi<1, 2> g_ext_gpio{EXT_SPI, EXT_LAT, g_ext_spi};

Timer g_tmm;

GpioAccessor g_extout0(EXT_OUT0, false, GPIO_OUTPUT);
GpioAccessor g_extin0(EXT_IN0, false, GPIO_INPUT);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);
  g_ext_spi.begin();
  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");
  Executor::instance()->begin();

  g_tmm.start_periodic(500);
}

void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
  if (g_tmm.check()) {
    Serial.printf("Input: %02x %d\n", g_ext_gpio.get_input_byte(0), g_extin0.read());
  }
  g_extout0.write(digitalRead(USER_BTN));
}
