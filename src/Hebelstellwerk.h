/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file Hebelstellwerk.h
 *
 * Main include file for the Hebelstellwerk library.
 *
 * @author Balazs Racz
 * @date 9 Mar 2024
 */

#include "utils/GpioRegistry.h"
#include "utils/Gpio.h"
#include "utils/Pwm.h"
#include "utils/PwmGpio.h"
#include "utils/ServoGpio.h"
#include "utils/AnalogGpio.h"
#include "utils/OrGpio.h"
#include "utils/Executor.h"
#include "utils/Timer.h"

#include "stw/TurnoutLever.h"
#include "stw/SignalLever.h"
#include "stw/RouteLever.h"
#include "stw/LockTable.h"
#include "stw/LeverKey.h"
#include "stw/I2CBlockImpl.h"
#include "stw/FelderBlock.h"
#include "stw/GlobalUnlocker.h"
#include "stw/BlockDetectorOverride.h"

GpioRegistry g_gpio_registry;
PwmRegistry g_pwm_registry;
TurnoutRegistry g_turnout_registry;
SignalRegistry g_signal_registry;
RouteRegistry g_route_registry;
BlockRegistry g_block_registry;

Executor ex;

const char* volatile g_death_file;
volatile int g_death_lineno;

using Signalhebel = SignalLever;
using Fahrstrassenhebel = RouteLever;
using Weichenhebel = TurnoutLever;
using Verschlusstabelle = LockTable;

#ifdef ARDUINO
#include "utils/ArduinoGpio.h"
#include "utils/Pwm9685.h"
#include "utils/Gpio23017.h"

ArduinoGpio g_arduino_gpio;
#endif

#if defined(USE_LOCONET) || defined(LOCONET_INCLUDED)
#include "utils/LnGpio.h"

#endif
