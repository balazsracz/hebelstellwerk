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
#include "utils/GpioDebug.h"
#include "utils/GpioCopier.h"


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


  HBL_F1A = INT_PORTA0,
  HBL_F1B = INT_PORTA1,
  HBL_F2A = INT_PORTA2,
  HBL_F2B = INT_PORTA3,
  HBL_F3A = INT_PORTA4,
  HBL_F3B = INT_PORTA5,
  HBL_F4A = INT_PORTA6,
  HBL_F4B = INT_PORTA7,
  HBL_F5A = INT_PORTB0,
  HBL_F5B = INT_PORTB1,
  HBL_F6A = INT_PORTB2,
  HBL_F6B = INT_PORTB3,

  EXT_SPI = 300,

  BRD_BLOCK_OUT = EXT_SPI + 0,
  TEMP0 = BRD_BLOCK_OUT - 1,
  LED_ENDF_ROT,
  LED_ENDF_WEISS,
  LED_ERLAUB_ROT,
  LED_ERLAUB_WEISS,
  LED_ANF_ROT,
  LED_ANF_WEISS,
  LED_FESTLEGE_ROT,
  LED_FESTLEGE_WEISS,


  EXT_SPI_IN = EXT_SPI + 8,

  BRD_BLOCK_IN = EXT_SPI_IN + 0,
  TEMP1 = BRD_BLOCK_IN - 1,
  KURBEL_RAW,
  BLOCK_IN1,
  BLOCK_IN2,
  BLOCK_IN3,
  BTN_ENDF,
  BTN_ERLAUB,
  BTN_ANF,
  BTN_FESTLEGE,

  BRD_HEBEL_IN_LINKS = BRD_BLOCK_IN + 8,
  TEMP2 = BRD_HEBEL_IN_LINKS - 1,
  HBL_W8,
  HBL_W9,
  HBL_W10_11,
  BTN_BELEG,
  SW_MODEUP,
  SW_MODEDN,
  HBL_SIGA,
  HBL_SIGB,

  BRD_HEBEL_IN_RECHTS = BRD_HEBEL_IN_LINKS + 8,
  TEMP3 = BRD_HEBEL_IN_RECHTS - 1,
  HBL_W1,
  HBL_W2a,
  HBL_W2b,
  HBL_W3,
  HBL_W4,
  HBL_W5_6,
  HBL_W7a,
  HBL_W7b,
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


static constexpr unsigned NUM_EXT_INPUT_REGISTERS = 3;
static constexpr unsigned NUM_EXT_OUTPUT_REGISTERS = 1;
GpioSpi<NUM_EXT_OUTPUT_REGISTERS, NUM_EXT_INPUT_REGISTERS> g_ext_gpio{EXT_SPI, EXT_LAT, g_ext_spi};

static constexpr unsigned NUM_INT_INPUT_REGISTERS = 2;
static constexpr unsigned NUM_INT_OUTPUT_REGISTERS = 1;
GpioSpi<NUM_INT_OUTPUT_REGISTERS, NUM_INT_INPUT_REGISTERS> g_int_gpio{INT_SPI, INT_LAT, g_int_spi};


std::initializer_list<GpioDebugInstance> g_gpios{
  {SW_MODEUP, "Special mode Up", true},
  {SW_MODEDN, "Special mode Down", true},
  {BTN_BELEG, "Belegtmelder Hilfstaste", true},
  {BTN_ANF, "Anfangsfeld taste", true},
  {BTN_ENDF, "Endfeld taste", true},
  {BTN_ERLAUB, "Erlaubnis taste", true},
  {BTN_FESTLEGE, "Festlege taste", true},
  {KURBEL_RAW, "Kurbel", true},
  {HBL_F1A, "Fahrstrasse 1a", true},
  {HBL_F1B, "Fahrstrasse 1b", true},
  {HBL_F2A, "Fahrstrasse 2a", true},
  {HBL_F2B, "Fahrstrasse 2b", true},
  {HBL_F3A, "Fahrstrasse 3a", true},
  {HBL_F3B, "Fahrstrasse 3b", true},
  {HBL_F4A, "Fahrstrasse 4a", true},
  {HBL_F4B, "Fahrstrasse 4b", true},
  {HBL_F5A, "Fahrstrasse 5a", true},
  {HBL_F5B, "Fahrstrasse 5b", true},
  {HBL_F6A, "Fahrstrasse 6a", true},
  {HBL_F6B, "Fahrstrasse 6b", true},
  {HBL_W1, "Weiche 1", true},
  {HBL_W2a, "Weiche 2a", true},
  {HBL_W2b, "Weiche 2b", true},
  {HBL_W3, "Weiche 3", true},
  {HBL_W4, "Weiche 4", true},
  {HBL_W5_6, "Weiche 5", true},
  {HBL_W7a, "Weiche 7a", true},
  {HBL_W7b, "Weiche 7b", true},
  {HBL_W8, "Weiche 8", true},
  {HBL_W9, "Weiche 9", true},
  {HBL_W10_11, "Weiche 10/11", true},
  {HBL_SIGA, "Signal A", true},
  {HBL_SIGB, "Signal B", true},
};


GpioDebug g_gpio_printer{g_gpios};

std::initializer_list<GpioCopyInstance> g_shadows{
  {BTN_ANF, LED_ANF_ROT, true},
  {BTN_ANF, LED_ANF_WEISS, false},
  {BTN_ENDF, LED_ENDF_ROT, true},
  {BTN_ENDF, LED_ENDF_WEISS, false},
  {BTN_ERLAUB, LED_ERLAUB_ROT, true},
  {BTN_ERLAUB, LED_ERLAUB_WEISS, false},
  {BTN_FESTLEGE, LED_FESTLEGE_ROT, true},
  {BTN_FESTLEGE, LED_FESTLEGE_WEISS, false},
};

GpioCopy g_gpio_shadow{g_shadows};



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
}

void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
