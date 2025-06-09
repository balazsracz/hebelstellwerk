/// Uses a Nucleo 303 with the OpenLCB IO Board extension to drive external SPI connected GPIOs.
/// The ionboard PORTA, PORTB is driven using SPI2, and external shift registers are driven
/// using SPI3. 

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include <SPI.h>
#include "utils/GpioSpi.h"


#ifndef ARDUINO
#error baaa
#endif

int EXT_LAT = PC11;

SPIClass g_ext_spi{PB5_ALT1, PB4_ALT1, PB3_ALT1};


SPIClass g_int_spi{/*mosi*/ PB15, /*miso*/ PB14, /*sck*/ PB13};

int INT_LAT = PA6;



enum GpioPin : gpio_pin_t {
  ONBOARD_LED = 13,
  ONBOARD_BTN = USER_BTN,

  INT_SPI = 250,
  INT_OUT_DUMMY = INT_SPI,
  INT_PORTB7 = INT_OUT_DUMMY + 8,
  INT_PORTB6,
  INT_PORTB5,
  INT_PORTB4,
  INT_PORTB3,
  INT_PORTB2,
  INT_PORTB1,
  INT_PORTB0,

  INT_PORTA7,
  INT_PORTA6,
  INT_PORTA5,
  INT_PORTA4,
  INT_PORTA3,
  INT_PORTA2,
  INT_PORTA1,
  INT_PORTA0,

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

static constexpr unsigned NUM_EXT_INPUT_REGISTERS = 1;
static constexpr unsigned NUM_EXT_OUTPUT_REGISTERS = 2;
GpioSpi<NUM_EXT_OUTPUT_REGISTERS, NUM_EXT_INPUT_REGISTERS> g_ext_gpio{EXT_SPI, EXT_LAT, g_ext_spi};

static constexpr unsigned NUM_INT_INPUT_REGISTERS = 2;
static constexpr unsigned NUM_INT_OUTPUT_REGISTERS = 1;
GpioSpi<NUM_INT_OUTPUT_REGISTERS, NUM_INT_INPUT_REGISTERS> g_int_gpio{INT_SPI, INT_LAT, g_int_spi};


Timer g_tmm;

GpioAccessor g_extout0(EXT_OUT0, false, GPIO_OUTPUT);
GpioAccessor g_extin0(INT_PORTA0, false, GPIO_INPUT);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);
  g_ext_spi.begin();
  g_int_spi.begin();
  pinMode(PB15, OUTPUT);
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
    Serial.printf("Input: %02x | %02x %02x %d\n", g_ext_gpio.get_input_byte(0), 
      g_int_gpio.get_input_byte(0), g_int_gpio.get_input_byte(1), 
      g_extin0.read());
  }
  g_extout0.write(digitalRead(USER_BTN));
}
