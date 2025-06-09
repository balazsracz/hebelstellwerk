/// Uses a Nucleo 303 with the OpenLCB IO Board extension to drive external SPI connected GPIOs.
/// The ionboard PORTA, PORTB is driven using SPI2, and external shift registers are driven
/// using SPI3. 

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include <SPI.h>
#include "utils/GpioSpi.h"

#include "stw/DirectBlock.h"
#include "stw/SimpleBlockUi.h"
#include "utils/AnalogDemux.h"
#include "utils/ArduinoStm32SpiPixel.h"
#include "utils/Blinker.h"
#include "utils/PixelGpio.h"


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

  BLOCK_DETECTOR = 100,
  
  ANALOG_BTN_START = 110,
  BTN_C_VORBLOCK = ANALOG_BTN_START,
  BTN_C_ABGABE,
  BTN_C_RUCKBLOCK,
  BTN_A_RUCKBLOCK,
  BTN_A_ABGABE,
  BTN_A_VORBLOCK,
  BTN_B_VORBLOCK,
  BTN_B_ABGABE,
  BTN_B_RUCKBLOCK,

  PIXEL_START = 120,
  PX_C_ANFANGSFELD = PIXEL_START,
  PX_C_ERLAUBNISFELD,
  PX_C_ENDFELD,

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

  EXT_SPI = 300,

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

GlobalState st;
GlobalUnlocker unlocker{ONBOARD_BTN, true};

static const int16_t kCenters[] = {913, 786, 683, 559, 461, 346, 254, 171, 93};
AnalogDemux gpio_an{110, PB0, kCenters, sizeof(kCenters) / sizeof(kCenters[0])};

HardwareSerial BlockCSerial(PB11 /*rx*/, PB10 /*tx*/);

SpiPixelStrip strip(6, PA7, PB4, PB3);
const uint32_t kOutputParams[] = {
    0, Pixel::RED, Pixel::WHITE,  //
    1, Pixel::RED, Pixel::WHITE,  //
    2, Pixel::RED, Pixel::WHITE,  //
};

PixelGpio px_gpio{&strip, 120, 6, kOutputParams};

DirectBlock block_c{"C", &BlockCSerial, PB11, PA15};
SimpleBlockUi c_ui{&block_c};
const uint16_t c_ui_rdy = c_ui.set_vorblock_taste(BTN_C_VORBLOCK, false) |
                          c_ui.set_ruckblock_taste(BTN_C_RUCKBLOCK, false) |
                          c_ui.set_abgabe_taste(BTN_C_ABGABE, false) |
                          c_ui.set_anfangsfeld(PX_C_ANFANGSFELD, false) |
                          c_ui.set_endfeld(PX_C_ENDFELD, false) |
                          c_ui.set_erlaubnisfeld(PX_C_ERLAUBNISFELD, false);


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

  //@todo check these
  pinMode(PC2, INPUT_PULLUP);
  pinMode(PA9, INPUT_PULLUP);
  // digitalWrite(PA9, HIGH);
  pinMode(PC5, OUTPUT);
  digitalWrite(PC5, LOW);
  // end todo

  Serial.begin(115200);
  // delay(100);
  Serial.println("Hello, world");
  Executor::instance()->begin();

  strip.set_brightness(0x20);

  ASSERT(c_ui_rdy == c_ui.EXPECTED_SETUP);

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
