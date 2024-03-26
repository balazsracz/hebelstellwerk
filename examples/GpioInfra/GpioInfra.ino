/** \copyright
 * Copyright (c) 2024, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file GpioInfra.ino
 *
 * Example Arduino sketch for the Hebelstellwerk library showing various GPIO
 * infrastructure including using GPIO extenders, PWM extenders and driving
 * Servos.
 *
 * @author Balazs Racz
 * @date 26 March 2024
 */

#include <Arduino.h>
#include <Hebelstellwerk.h>

#include "utils/Blinker.h"

PWM9685 pwm_chip(16);
PwmGpio gpio_1(101, 18 /*pwm channel 2*/, 10, 60);
ServoGpio gpio_2(102, 20 /*pwm channel 4*/, 10, 30, 400);

Gpio23017 ext_chip(128);
GpioAccessor acco(128, false, GPIO_OUTPUT);
GpioAccessor acci(128+8, false, GPIO_INPUT);

Blinker blinker{101};
Blinker blinker2{LED_BUILTIN};
Blinker blinker3{102};
//Blinker blinker4{128};

/// Arduino setup routine.
void setup() {
  Serial.begin(9600);
  Serial.println("hello world");
  // Calls the executor to do begin for all registered objects.
  ex.begin();
}

/// Arduino loop routine.
void loop() {
  // Calls the executor to do loop for all registered objects.
  ex.loop();
  acco.write(acci.read());
}
