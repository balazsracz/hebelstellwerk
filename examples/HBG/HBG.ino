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
#include "utils/ArduinoStm32DmaPwm.h"
#include "utils/Pwm.h"
#include "utils/CommandHandler.h"


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
  //BLOCK_DETECTOR_HW,
  DUMMY,

  LED_STOERUNGSMELDER = DUMMY,

  SND_START = 105,
  SND_1 = SND_START,
  SND_5,
  
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

  SRV_START = 150 - 1,
  SRV_LOCK_F1a,
  SRV_LOCK_F1b,
  SRV_LOCK_F2a,
  SRV_LOCK_F2b,
  SRV_LOCK_F3a,
  SRV_LOCK_F3b,
  SRV_LOCK_F4a,
  SRV_LOCK_F4b,
  SRV_LOCK_F5a,
  SRV_LOCK_F5b,
  SRV_LOCK_F6a,
  SRV_LOCK_F6b,

  SRV_LOCK_W1,
  SRV_LOCK_W2a, 
  SRV_LOCK_W2b,
  SRV_LOCK_W3,
  SRV_LOCK_W4,
  SRV_LOCK_W5_6,
  SRV_LOCK_W7a,
  SRV_LOCK_W7b,
  SRV_LOCK_W8,
  SRV_LOCK_W9,
  SRV_LOCK_W10_11,
  SRV_LOCK_SIGA,
  SRV_LOCK_SIGB,
  
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


  HBL_F1a = INT_PORTA0,
  HBL_F1b = INT_PORTA1,
  HBL_F2a = INT_PORTA2,
  HBL_F2b = INT_PORTA3,
  HBL_F3a = INT_PORTA4,
  HBL_F3b = INT_PORTA5,
  HBL_F4a = INT_PORTA6,
  HBL_F4b = INT_PORTA7,
  HBL_F5a = INT_PORTB0,
  HBL_F5b = INT_PORTB1,
  HBL_F6a = INT_PORTB2,
  HBL_F6b = INT_PORTB3,
  BLOCK_DETECTOR_HW = INT_PORTB4,

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

  BRD_BLOCK_BACK_OUT = BRD_BLOCK_OUT + 8,
  TEMP5 = BRD_BLOCK_BACK_OUT - 1,
  LED_STRECKENTASTENSPERRE,
  LED_HALTSTELLMELDER,
  SND_1_HW,
  SND_5_HW,
  

  EXT_SPI_IN = EXT_SPI + 16,

  BRD_BLOCK_IN = EXT_SPI_IN + 0,
  TEMP1 = BRD_BLOCK_IN - 1,
  KURBEL_RAW,
  BLOCK_IN1,
  BLOCK_IN2,
  BTN_NC1,
  BTN_ENDF,
  BTN_ERLAUB,
  BTN_ANF,
  BTN_FESTLEGE,

  BRD_BLOCK_BACK_IN = BRD_BLOCK_IN + 8,
  TEMP4 = BRD_BLOCK_BACK_IN - 1,
  BTN_FHT,
  
  
  BRD_HEBEL_IN_LINKS = BRD_BLOCK_BACK_IN + 8,
  TEMP2 = BRD_HEBEL_IN_LINKS - 1,
  HBL_W2b,
  HBL_W2a,
  HBL_W1,
  BTN_BELEG,
  SW_MODEUP,
  SW_MODEDN,
  HBL_SIGB,
  HBL_SIGA,

  BRD_HEBEL_IN_RECHTS = BRD_HEBEL_IN_LINKS + 8,
  TEMP3 = BRD_HEBEL_IN_RECHTS - 1,
  HBL_W10_11,
  HBL_W9,
  HBL_W8,
  HBL_W7b,
  HBL_W7a,
  HBL_W5_6,
  HBL_W4,
  HBL_W3,
};

enum PwmPin : pwm_pin_t {
  PWM_DMA = 100,

  PWM_LOCK_F1a = PWM_DMA,
  PWM_LOCK_F1b,
  PWM_LOCK_F2a,
  PWM_LOCK_F2b,
  PWM_LOCK_F3a,
  PWM_LOCK_F3b,
  PWM_LOCK_F4a,
  PWM_LOCK_F4b,
  PWM_LOCK_F5a,
  PWM_LOCK_F5b,
  PWM_LOCK_F6a,
  PWM_LOCK_F6b,

  PWM_LOCK_W10_11,
  PWM_LOCK_W9,
  PWM_LOCK_W8,
  PWM_LOCK_W7b,
  PWM_LOCK_W7a,
  PWM_LOCK_W5_6,
  PWM_LOCK_W4,
  PWM_LOCK_W3,
  PWM_LOCK_W2b,
  PWM_LOCK_W2a, 
  PWM_LOCK_W1,
  PWM_LOCK_SIGB,
  PWM_LOCK_SIGA,
};

DmaPwm g_pwm(PWM_DMA, {PC8, PC9, PC6,  PC5,  PA12, PA11, PB12, PC7,  PB2,
                       PA8, PB1, PB15, PC10, PC12, PD2,  PC14, PC15, PA0,
                       PA1, PF1, PC0,  PC3,  PC1,  PC2,  PA4});

// was: PC9, PC8, PC6, PA12, PA11, PB12, PC7, PB2, PA8, PB1, PB15, PC2, PC5,
// PC10, PC12, PD2, PC14, PC15, PA0, PA1, PF1, PA4, PC1, PC3, PC0

DummyGpio gpio_dummy(DUMMY);

/// Milliseconds how long it should take to transition a servo from on to off.
static constexpr unsigned XN_TIME = 500;

ServoGpio servos[25] = {
    {SRV_LOCK_F1a, PWM_LOCK_F1a, 30, 90, XN_TIME},
    {SRV_LOCK_F1b, PWM_LOCK_F1b, 160, 100, XN_TIME},
    {SRV_LOCK_F2a, PWM_LOCK_F2a, 10, 70, XN_TIME},
    {SRV_LOCK_F2b, PWM_LOCK_F2b, 155, 90, XN_TIME},
    {SRV_LOCK_F3a, PWM_LOCK_F3a, 30, 90, XN_TIME},
    {SRV_LOCK_F3b, PWM_LOCK_F3b, 140, 80, XN_TIME},
    {SRV_LOCK_F4a, PWM_LOCK_F4a, -50, 10, XN_TIME},
    {SRV_LOCK_F4b, PWM_LOCK_F4b, 215, 155, XN_TIME},
    {SRV_LOCK_F5a, PWM_LOCK_F5a, 0, 80, XN_TIME},
    {SRV_LOCK_F5b, PWM_LOCK_F5b, 160, 80, XN_TIME},
    {SRV_LOCK_F6a, PWM_LOCK_F6a, 30, 110, XN_TIME},
    {SRV_LOCK_F6b, PWM_LOCK_F6b, 150, 90, XN_TIME},

    {SRV_LOCK_W10_11, PWM_LOCK_W10_11, 45, 100, XN_TIME},
    {SRV_LOCK_W9, PWM_LOCK_W9, 30, 85, XN_TIME},
    {SRV_LOCK_W8, PWM_LOCK_W8, 40, 90, XN_TIME},
    {SRV_LOCK_W7b, PWM_LOCK_W7b, 50, 100, XN_TIME},  // 4
    {SRV_LOCK_W7a, PWM_LOCK_W7a, 35, 95, XN_TIME},   // 5
    {SRV_LOCK_W5_6, PWM_LOCK_W5_6, 25, 85, XN_TIME},
    {SRV_LOCK_W4, PWM_LOCK_W4, 45, 95, XN_TIME},
    {SRV_LOCK_W3, PWM_LOCK_W3, 55, 110, XN_TIME}, //8
    {SRV_LOCK_W2b, PWM_LOCK_W2b, 60, 110, XN_TIME},  //9
    {SRV_LOCK_W2a, PWM_LOCK_W2a, 60, 115, XN_TIME},
    {SRV_LOCK_W1, PWM_LOCK_W1, 60, 115, XN_TIME},
    {SRV_LOCK_SIGB, PWM_LOCK_SIGB, 65, 115, XN_TIME}, // 12
    {SRV_LOCK_SIGA, PWM_LOCK_SIGA, 60, 110, XN_TIME}, // 13
};

void CommandHandler::set_servo(int servo_num, int degrees) {
  if (servo_num < 1 || servo_num > ARRAYSIZE(servos)) {
    Serial.println(F("Error: Invalid servo number"));
    return;
  }
  if (degrees < -90 || degrees > 270) {
    Serial.println(F("Error: Invalid servo degree"));
    return;
  }
  servos[servo_num - 1].set_manual_degree(degrees);
}

void CommandHandler::set_gpio(int gpio_num, bool is_on) {
  const Gpio* gpio = GpioRegistry::instance()->get_or_null(gpio_num);
  if (!gpio) {
    Serial.println(F("Error: Gpio number not registered"));
    return;
  }
  gpio->write(gpio_num, is_on);
}

SpiPixelStrip strip(6, PA7, PB4, PB3);
const uint32_t kOutputParams[] = {
    0, Pixel::RED, Pixel::WHITE,  //
    1, Pixel::RED, Pixel::WHITE,  //
    2, Pixel::RED, Pixel::WHITE,  //
};

PixelGpio px_gpio{&strip, 120, 6, kOutputParams};

static constexpr unsigned NUM_EXT_INPUT_REGISTERS = 4;
static constexpr unsigned NUM_EXT_OUTPUT_REGISTERS = 2;
GpioSpi<NUM_EXT_OUTPUT_REGISTERS, NUM_EXT_INPUT_REGISTERS> g_ext_gpio{EXT_SPI, EXT_LAT, g_ext_spi};

static constexpr unsigned NUM_INT_INPUT_REGISTERS = 2;
static constexpr unsigned NUM_INT_OUTPUT_REGISTERS = 1;
GpioSpi<NUM_INT_OUTPUT_REGISTERS, NUM_INT_INPUT_REGISTERS> g_int_gpio{INT_SPI, INT_LAT, g_int_spi};

CommandHandler cli;

GlobalState st;
GlobalUnlocker unlocker{SW_MODEDN, true};

static const int16_t kCenters[] = {913, 786, 683, 559, 461, 346, 254, 171, 93};
AnalogDemux gpio_an{110, PB0, kCenters, sizeof(kCenters) / sizeof(kCenters[0])};

HardwareSerial BlockCSerial(PB11 /*rx*/, PB10 /*tx*/);

DirectBlock block{"C", &BlockCSerial, PB11, PA15};
SimpleBlockUi block_ui{&block};
const uint16_t ui_rdy = block_ui.set_vorblock_taste(BTN_C_VORBLOCK, false) |
                        block_ui.set_ruckblock_taste(BTN_C_RUCKBLOCK, false) |
                        block_ui.set_abgabe_taste(BTN_C_ABGABE, false) |
                        block_ui.set_anfangsfeld(PX_C_ANFANGSFELD, false) |
                        block_ui.set_endfeld(PX_C_ENDFELD, false) |
                        block_ui.set_erlaubnisfeld(PX_C_ERLAUBNISFELD, false);


std::initializer_list<GpioDebugInstance> g_gpios{
  {SW_MODEUP, "Special mode Up", true},
  {SW_MODEDN, "Special mode Down", true},
  {BTN_BELEG, "Belegtmelder Hilfstaste", true},
  {BTN_ANF, "Anfangsfeld taste", true},
  {BTN_ENDF, "Endfeld taste", true},
  {BTN_ERLAUB, "Erlaubnis taste", true},
  {BTN_FESTLEGE, "Festlege taste", true},
  {KURBEL_RAW, "Kurbel", true},
  {HBL_F1a, "Fahrstrasse 1a", true},
  {HBL_F1b, "Fahrstrasse 1b", true},
  {HBL_F2a, "Fahrstrasse 2a", true},
  {HBL_F2b, "Fahrstrasse 2b", true},
  {HBL_F3a, "Fahrstrasse 3a", true},
  {HBL_F3b, "Fahrstrasse 3b", true},
  {HBL_F4a, "Fahrstrasse 4a", true},
  {HBL_F4b, "Fahrstrasse 4b", true},
  {HBL_F5a, "Fahrstrasse 5a", true},
  {HBL_F5b, "Fahrstrasse 5b", true},
  {HBL_F6a, "Fahrstrasse 6a", true},
  {HBL_F6b, "Fahrstrasse 6b", true},
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
  {LED_ANF_WEISS, LED_ANF_ROT, true},
  {LED_ENDF_WEISS, LED_ENDF_ROT, true},
  {LED_ERLAUB_WEISS, LED_ERLAUB_ROT, true},
  {LED_FESTLEGE_WEISS, LED_FESTLEGE_ROT, true},
  // Servos für Fahrstrassenhebel sind gleich.
  {SRV_LOCK_F1a, SRV_LOCK_F1b, false},
  {SRV_LOCK_F2a, SRV_LOCK_F2b, false},
  {SRV_LOCK_F3a, SRV_LOCK_F3b, false},
  {SRV_LOCK_F4a, SRV_LOCK_F4b, false},
  {SRV_LOCK_F5a, SRV_LOCK_F5b, false},
  {SRV_LOCK_F6a, SRV_LOCK_F6b, false},
};

GpioCopy g_gpio_shadow{g_shadows};

// The Hilfsbelegungstaste is active low. The real hardware is also active low.
// The output is active high (true means occupied).
BlockDetectorOverride gpio_det_ab(BLOCK_DETECTOR, BTN_BELEG, true,
                                  BLOCK_DETECTOR_HW, true);


// ==================== logical devices ================

enum SignalId : uint8_t { SIGNAL_A_EIN, SIGNAL_B_AUS };

enum TurnoutId : uint8_t { W1, W2a, W2b, W3, W4, W5_6, W7a, W7b, //
  W8, W9, W10_11 };

enum RouteId : uint8_t { a1, b1, a2, b2, a3, b3, a4, b4, a5, b5, a6, b6 };

enum BlockId : uint8_t {
  BLOCK_AB,
};

Weichenhebel TW1{W1, HBL_W1, true, SRV_LOCK_W1, false};
Weichenhebel TW2a{W2a, HBL_W2a, true, SRV_LOCK_W2a, false};
Weichenhebel TW2b{W2b, HBL_W2b, true, SRV_LOCK_W2b, false};
Weichenhebel TW3{W3, HBL_W3, true, SRV_LOCK_W3, false};
Weichenhebel TW4{W4, HBL_W4, true, SRV_LOCK_W4, false};
Weichenhebel TW5_6{W5_6, HBL_W5_6, true, SRV_LOCK_W5_6, false};
Weichenhebel TW7a{W7a, HBL_W7a, true, SRV_LOCK_W7a, false};
Weichenhebel TW7b{W7b, HBL_W7b, true, SRV_LOCK_W7b, false};
Weichenhebel TW8{W8, HBL_W8, true, SRV_LOCK_W8, false};
Weichenhebel TW9{W9, HBL_W9, true, SRV_LOCK_W9, false};
Weichenhebel TW10_11{W10_11, HBL_W10_11, true, SRV_LOCK_W10_11, false};

Signalhebel SA{SIGNAL_A_EIN, HP2, HBL_SIGA, false, SRV_LOCK_SIGA, false};
Signalhebel SB{SIGNAL_B_AUS, HP1, HBL_SIGB, false, SRV_LOCK_SIGB, false};

Fahrstrassenhebel Rab1{a1,      b1,   HBL_F1a,      true,
                       HBL_F1b, true, SRV_LOCK_F1a, false};
Fahrstrassenhebel Rab2{a2,      b2,   HBL_F2a,      true,
                       HBL_F2b, true, SRV_LOCK_F2a, false};
Fahrstrassenhebel Rab3{a3,      b3,   HBL_F3a,      true,
                       HBL_F3b, true, SRV_LOCK_F3a, false};
Fahrstrassenhebel Rab4{a4,      b4,   HBL_F4a,      true,
                       HBL_F4b, true, SRV_LOCK_F4a, false};
Fahrstrassenhebel Rab5{a5,      b5,   HBL_F5a,      true,
                       HBL_F5b, true, SRV_LOCK_F5a, false};
Fahrstrassenhebel Rab6{a6,      b6,   HBL_F6a,      true,
                       HBL_F6b, true, SRV_LOCK_F6a, false};

#if 1

FelderBlock blk_ab(&block, BLOCK_AB, BLOCK_DETECTOR, false, BTN_FESTLEGE, true,
                   LED_FESTLEGE_WEISS, true, BTN_FHT, true);

const auto abrdy = blk_ab.set_vorblock_taste(BTN_ANF, true) |
                   blk_ab.set_ruckblock_taste(BTN_ENDF, true) |
                   blk_ab.set_abgabe_taste(BTN_ERLAUB, true) |
                   blk_ab.set_kurbel(KURBEL_RAW, true) |
                   blk_ab.set_anfangsfeld(LED_ANF_WEISS, true) |
                   blk_ab.set_endfeld(LED_ENDF_WEISS, true) |
                   blk_ab.set_erlaubnisfeld(LED_ERLAUB_WEISS, true) |
                   blk_ab.set_signalhaltmelder(LED_HALTSTELLMELDER, false) |
                   blk_ab.set_storungsmelder(LED_STOERUNGSMELDER, false) | // does not exist
                   blk_ab.set_streckentastensperre(LED_STRECKENTASTENSPERRE, false);

#endif

// ================= Verschlusstabelle ======================

Verschlusstabelle vtbl({
    Fstr(a1),
    WeichePlus(W1), WeicheMinus(W2a), WeicheMinus(W2b), WeichePlus(W3),
    WeicheMinus(W4), WeichePlus(W5_6), WeichePlus(W7b),
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b1),
    WeichePlus(W1), WeicheMinus(W2a), WeicheMinus(W2b), WeichePlus(W3),
    WeicheMinus(W4), WeichePlus(W5_6), WeichePlus(W7b),
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),

    Fstr(a2),
    WeichePlus(W1), WeicheMinus(W2a), WeicheMinus(W2b), WeicheMinus(W3),
    WeichePlus(W4), WeichePlus(W5_6), WeichePlus(W7b),
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b2),
    WeichePlus(W1), WeicheMinus(W2a), WeicheMinus(W2b), WeicheMinus(W3),
    WeichePlus(W4), WeichePlus(W5_6), WeichePlus(W7b),
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),

    Fstr(a3),
    WeichePlus(W1), WeichePlus(W2a), WeicheMinus(W2b),
    WeichePlus(W4), WeichePlus(W7b),
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b3),
    WeichePlus(W1), WeichePlus(W2a), WeicheMinus(W2b),
    WeichePlus(W4), WeichePlus(W7b),
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),

    Fstr(a4),
    WeicheMinus(W1), WeichePlus(W2b), WeichePlus(W7a), WeicheMinus(W7b),
    WeichePlus(W9), 
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b4),
    WeicheMinus(W1), WeichePlus(W2b), WeichePlus(W7a), WeicheMinus(W7b),
    WeichePlus(W9), 
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),

    Fstr(a5),
    WeicheMinus(W1), WeichePlus(W2b), WeicheMinus(W7a), WeicheMinus(W7b),
    WeicheMinus(W8), WeichePlus(W9), WeichePlus(W10_11),
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b5),
    WeicheMinus(W1), WeichePlus(W2b), WeicheMinus(W7a), WeicheMinus(W7b),
    WeicheMinus(W8), WeichePlus(W9), WeichePlus(W10_11),
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),
    
    Fstr(a6),
    WeicheMinus(W1), WeichePlus(W2b), WeicheMinus(W7a), WeicheMinus(W7b),
    WeichePlus(W8), WeicheMinus(W9), WeichePlus(W10_11),
    Hp2(SIGNAL_A_EIN), BlockEinfahrt(BLOCK_AB),
    Fstr(b6),
    WeicheMinus(W1), WeichePlus(W2b), WeicheMinus(W7a), WeicheMinus(W7b),
    WeichePlus(W8), WeicheMinus(W9), WeichePlus(W10_11),
    Hp1(SIGNAL_B_AUS), BlockAusfahrt(BLOCK_AB),
  });



class SndGpio : public Gpio, private Executable {
 public:
  SndGpio(gpio_pin_t gpio_pin, gpio_pin_t hw)
      : gpio_pin_(gpio_pin)
      , hw_pin_(hw, true, GPIO_OUTPUT)
  {
    GpioRegistry::instance()->register_obj(this, gpio_pin);
    Executor::instance()->add(this);
  }

  void write(gpio_pin_t pin, bool value) const override {
    if (!value) return;
    hw_pin_.write(true);
    tm_.start_oneshot(100);
  }

  bool read(gpio_pin_t pin) const override {
    return false;
  }
  
  void begin() override {
    hw_pin_.write(false);
  }

  void loop() override {
    if (tm_.check()) {
      hw_pin_.write(false);
    }
  }
  
 private:
  gpio_pin_t gpio_pin_;
  GpioAccessor hw_pin_;
  mutable Timer tm_;
};

SndGpio snd1(SND_1, SND_1_HW);
SndGpio snd2(SND_5, SND_5_HW);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(USER_BTN, INPUT_PULLUP);
  g_ext_spi.begin();
  g_int_spi.begin();

  pinMode(PA9, INPUT_PULLUP); // LN-TX

  Serial.begin(115200);
  // delay(100);
  Serial.println("\n\n=================\nHello, world");
  Executor::instance()->begin();

  strip.set_brightness(0x20);

  ASSERT(ui_rdy == block_ui.EXPECTED_SETUP);
  blk_ab.check_setup(abrdy);

  // SPI1 MOSI that is unused and we ahve a servo on that pin.
  pinMode(PB15, OUTPUT);
  digitalWrite(PB15, LOW);
  // Idk
  pinMode(PC5, OUTPUT);
  digitalWrite(PC5, LOW);

  block.set_sound(DirectBlock::SND_RUCKBLOCKEN_IN, SND_5)
      .set_sound(DirectBlock::SND_ERLAUBNIS_ABGABE_IN, SND_1)
      .set_sound(DirectBlock::SND_VORBLOCKEN_IN, SND_1);
}

void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
}
