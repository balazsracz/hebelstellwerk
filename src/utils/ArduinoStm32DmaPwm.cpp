/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file ArduinoStm32DmaPwm.h
 *
 * Drives output pins using DMA-based PWM, one at a time. Intended for servos.
 *
 * @author Balazs Racz
 * @date 12 Jun 2025
 */

#include "utils/ArduinoStm32DmaPwm.h"

#include "utils/Executor.h"
#include "utils/Macros.h"
#include "utils/Pwm.h"
#include "utils/Singleton.h"

#include "stm32yyxx_hal_conf.h"

#define DMA_INSTANCE               DMA1_Channel1
#define DMA_INSTANCE_IRQ           DMA1_Channel1_IRQn
#define DMA_INSTANCE_IRQHANDLER    DMA1_Channel1_IRQHandler

DMA_HandleTypeDef     g_dma_handle = {0};
static void transfer_complete(DMA_HandleTypeDef *handle);

class DmaPwmImpl : public Executable, public Pwm, public Singleton<DmaPwmImpl> {
 public:
  DmaPwmImpl(pwm_pin_t pwm_pin_start, std::initializer_list<int> pins)
      : start_pin_(pwm_pin_start), pins_(pins) {
    Executor::instance()->add(this);
    PwmRegistry::instance()->register_obj(this, start_pin_,
                                          Count{pins_.size()});
  }

  /// @return how many ticks this object has for a one millisecond pulse
  /// width. We run at a fixed 1 MHz.
  tick_t tick_per_msec() const override { return 1000; }

  /// @return period length in tick count. We have a fixed 20 msec period.
  tick_t tick_per_period() const override { return 20000; }

  /// Update the output pulse width.
  /// @param pin the output pin to act upon.
  /// @param count_high how many counts of the period should the output be high.
  void write(pwm_pin_t pin, tick_t count_high) const override {
    pin -= start_pin_;
    ASSERT(pin < pins_.size());
    pins_[pin].count_high_ = count_high_;
  }

  void begin() override {
    // We purposefully use drifting here instead of periodic, because if we
    // have too many outputs active, we might not make it around all of them in
    // 20 msec.
    tm_.start_drifting(20);
  }

  void loop() override {
    if (next_active_pin_ >= pins_.size() && tm_.check()) {
      // Start new iteration.
      next_active_pin_ = 0;
    }
  }
  
 private:
  /// Starts the DMA based output for a given pin.
  /// @param index 0 to pins_.size() - 1.
  void start_dma(unsigned index) {
    uint32_t gpio_base = 0;
    unsigned pin_number = 13; // 0 to 15
  }
  
  struct PinInfo {
    PinInfo(int ardino_pin) {}
    
    int arduino_pin_;
    /// How many counts this output should be high during a period. Zero means
    /// the pin is inactive.
    uint32_t count_high_{0};
  };
  std::vector<PinInfo> pins_;
  /// Pin number of the first pin we care about.
  pwm_pin_t start_pin_;
  /// During the iteration, what is the next pin number to act upon.
  uint8_t next_active_pin_{-1};
  /// True if the pulse generation on the current pin is completed.
  bool current_pin_done_{true};
  /// Timer that helps keeping to the 20 msec refresh cycle.
  Timer tm_;
  /// This memory will be transferred to the GPIO peripheral to turn the needed
  /// pin on and off.
  uint32_t dma_buffer_[2];
};  // class DmaPwmImpl

void DmaPwm::create_impl(pwm_pin_t pin_start, std::initializer_list<int> pins) {
  impl_ = new DmaPwmImpl(pin_start, pins);
}


static void transfer_complete(DMA_HandleTypeDef *g_dma_handle) {
  
}


void DMA_INSTANCE_IRQHANDLER(void)
{
  /* Check the interrupt and clear flag */
  HAL_DMA_IRQHandler(&g_dma_handle);
}

