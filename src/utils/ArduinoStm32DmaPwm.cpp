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

#include <Arduino.h>

#include "utils/Executor.h"
#include "utils/Macros.h"
#include "utils/Pwm.h"
#include "utils/Timer.h"
#include "utils/Singleton.h"

#include "stm32yyxx_hal_conf.h"

#define DMA_INSTANCE               DMA1_Channel1

DMA_HandleTypeDef     g_dma_handle = {0};

#define TIMER_INSTANCE               TIM2


class DmaPwmImpl : public Executable, public Pwm, public Singleton<DmaPwmImpl> {
 public:
  DmaPwmImpl(pwm_pin_t pwm_pin_start, std::initializer_list<int> pins)
      : start_pin_(pwm_pin_start) {
    Executor::instance()->add(this);
    pins_.reserve(pins.size());
    for (int p : pins) {
      pins_.emplace_back(p);
    }
    PwmRegistry::instance()->register_obj(this, start_pin_,
                                          Count{(uint16_t)pins_.size()});
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
    pins_[pin].count_high_ = count_high;
  }

  void begin() override {
    // We purposefully use drifting here instead of periodic, because if we
    // have too many outputs active, we might not make it around all of them in
    // 20 msec.
    tm_.start_drifting(20);

    // Sets up pin names and initializes outputs.
    for (auto& p : pins_) {
      p.pin_name_ = digitalPinToPinName(p.arduino_pin_);
      digitalWriteFast(p.pin_name_, LOW);
      pinMode(p.arduino_pin_, OUTPUT);
    }
    
    // Sets up HAL DMA configuration.
    DMA_InitTypeDef &dma_init = g_dma_handle.Init;
    dma_init.Direction = DMA_MEMORY_TO_PERIPH;
    dma_init.PeriphInc = DMA_PINC_DISABLE;
    dma_init.MemInc = DMA_MINC_ENABLE;
    dma_init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    dma_init.Mode = DMA_NORMAL;
    // Pick high priority to ensure jitter-free timing.
    dma_init.Priority = DMA_PRIORITY_VERY_HIGH;
    g_dma_handle.Parent = this;
    g_dma_handle.Instance = DMA_INSTANCE;

    auto ret = HAL_DMA_Init(&g_dma_handle);
    ASSERT(ret == HAL_OK);

    // Sets up HAL Timer configuration.
    timer_handle_.Instance = TIMER_INSTANCE;
    // Prescaler for 1000 timer ticks per millisecond.
    timer_handle_.Init.Prescaler = F_CPU / 1000000;
    timer_handle_.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    timer_handle_.Init.Period = 1000; // 1 msec, will set later.
    timer_handle_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
    timer_handle_.Init.RepetitionCounter = 0;
    timer_handle_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    ret = HAL_TIM_Base_Init(&timer_handle_);
    ASSERT(ret == HAL_OK);
  }

  void loop() override {
    // check for dma transfer complete
    if (pulse_in_progress_ && (__HAL_DMA_GET_COUNTER(&g_dma_handle) == 0)) {
      // Transfer seems to be done.
      stop_timer();
      ++next_active_pin_;
    }
    if (next_active_pin_ < pins_.size()) {
      if (!pulse_in_progress_) {
        // start pulse for next pin.
        if (!start_timer(next_active_pin_)) {
          ++next_active_pin_;
        }
      }
    } else if (tm_.check()) {
      // Start new iteration.
      next_active_pin_ = 0;
    }
  }

 private:
  /// Starts the DMA based output for a given pin.
  /// @param index 0 to pins_.size() - 1.
  bool start_timer(unsigned index) {
    if (!pins_[index].count_high_) {
      return false;
    }
    auto& pn = pins_[index].pin_name_;
    GPIO_TypeDef* port = get_GPIO_Port(STM_PORT(pn));
    uint32_t target_address = (uint32_t) &port->BSRR;
    dma_buffer_[0] = STM_LL_GPIO_PIN(pn); // bit set
    dma_buffer_[1] = dma_buffer_[0] << 16; // bit reset
    pulse_in_progress_ = true;
    auto ret =
        HAL_DMA_Start(&g_dma_handle, (uint32_t)dma_buffer_, target_address, 2);
    ASSERT(HAL_OK == ret);
    // Sets up timer.
    __HAL_TIM_SET_COUNTER(&timer_handle_, 1000);
    __HAL_TIM_SET_AUTORELOAD(&timer_handle_, pins_[index].count_high_);
    __HAL_TIM_ENABLE_DMA(&timer_handle_, TIM_DMA_UPDATE);    
    __HAL_TIM_ENABLE(&timer_handle_);
    // Triggering an update will perform one DMA right now and 
    HAL_TIM_GenerateEvent(&timer_handle_, TIM_EVENTSOURCE_UPDATE);

    return true;
  }

  /// Stops the DMA and resets the timer.
  void stop_timer() {
    __HAL_TIM_DISABLE_DMA(&timer_handle_, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE(&timer_handle_);
    // Calls the HAL to ensure that the state management is correct.
    auto ret = HAL_DMA_PollForTransfer(&g_dma_handle, HAL_DMA_FULL_TRANSFER, 0);
    ASSERT(ret == HAL_OK);
    pulse_in_progress_ = false;
  }

  struct PinInfo {
    PinInfo(int ardino_pin) {}
    
    int arduino_pin_;
    PinName pin_name_;
    /// How many counts this output should be high during a period. Zero means
    /// the pin is inactive.
    mutable uint32_t count_high_{0};
  };
  std::vector<PinInfo> pins_;
  /// Pin number of the first pin we care about.
  pwm_pin_t start_pin_;
  /// During the iteration, what is the next pin number to act upon.
  uint8_t next_active_pin_{0xff};
  /// True if there is an ongoing pulse generation for the current pin.
  bool pulse_in_progress_{false};
  /// Timer that helps keeping to the 20 msec refresh cycle.
  Timer tm_;
  /// This memory will be transferred to the GPIO peripheral to turn the needed
  /// pin on and off.
  uint32_t dma_buffer_[2];

  // HAL configuration and state
  TIM_HandleTypeDef timer_handle_{0};
};  // class DmaPwmImpl

void DmaPwm::create_impl(pwm_pin_t pin_start, std::initializer_list<int> pins) {
  impl_ = new DmaPwmImpl(pin_start, pins);
}
