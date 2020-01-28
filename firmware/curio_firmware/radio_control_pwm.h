/*   
 *   Copyright (c) 2019 Rhys Mainwaring
 *   All rights reserved
 *    
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *   1.  Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 * 
 *   2.  Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 * 
 *   3.  Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *  
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef CURIO_FIRMWARE_RADIO_CONTROL_PWM_H_
#define CURIO_FIRMWARE_RADIO_CONTROL_PWM_H_

#include "ros.h"

#include <curio_msgs/Channels.h>

#include <EnableInterrupt.h>

// The number of channels to decode
#define NUM_CHANNELS  12

// Interrupt pins for radio control channels
#define CH_PIN0       2
#define CH_PIN1       3
#define CH_PIN2       4
#define CH_PIN3       5
#define CH_PIN4       6
#define CH_PIN5       7

namespace curio_firmware {

  namespace radio_control {
    /*
     * Interrupt service routines to monitor the radio control pulse width
     */
    volatile int pwm0 = 0;
    volatile int pwm1 = 0;
    volatile int pwm2 = 0;
    volatile int pwm3 = 0;
    volatile int pwm4 = 0;
    volatile int pwm5 = 0;
    
    unsigned long pwm_time0 = 0;
    unsigned long pwm_time1 = 0;
    unsigned long pwm_time2 = 0;
    unsigned long pwm_time3 = 0;
    unsigned long pwm_time4 = 0;
    unsigned long pwm_time5 = 0;
    
    bool pwm_last0 = false;
    bool pwm_last1 = false;
    bool pwm_last2 = false;
    bool pwm_last3 = false;
    bool pwm_last4 = false;
    bool pwm_last5 = false;
    
    void detect_pwm_isr0() {
      // Read the state of the pin and check if it has changed.
      bool newer = digitalRead(CH_PIN0);
      if (pwm_last0 == newer) return;
      pwm_last0 = newer;
    
      // If the pwm pin is set HIGH start timing.
      if (newer) {
        pwm_time0 = micros();
      }
      // If the pwm pin is set LOW stop timing and record the pulse width.
      else {
        pwm0 = micros() - pwm_time0;
      }
    }
    
    void detect_pwm_isr1() {
      bool newer = digitalRead(CH_PIN1);
      if (pwm_last1 == newer) return;
      pwm_last1 = newer;
    
      if (newer) {
        pwm_time1 = micros();
      }
      else {
        pwm1 = micros() - pwm_time1;
      }
    }
  
    void detect_pwm_isr2() {
      bool newer = digitalRead(CH_PIN2);
      if (pwm_last2 == newer) return;
      pwm_last2 = newer;
    
      if (newer) {
        pwm_time2 = micros();
      }
      else {
        pwm2 = micros() - pwm_time2;
      }
    }
  
    void detect_pwm_isr3() {
      bool newer = digitalRead(CH_PIN3);
      if (pwm_last3 == newer) return;
      pwm_last3 = newer;
    
      if (newer) {
        pwm_time3 = micros();
      }
      else {
        pwm3 = micros() - pwm_time3;
      }
    }
  
    void detect_pwm_isr4() {
      bool newer = digitalRead(CH_PIN4);
      if (pwm_last4 == newer) return;
      pwm_last4 = newer;
    
      if (newer) {
        pwm_time4 = micros();
      }
      else {
        pwm4 = micros() - pwm_time4;
      }
    }
  
    void detect_pwm_isr5() {
      bool newer = digitalRead(CH_PIN5);
      if (pwm_last5 == newer) return;
      pwm_last5 = newer;
    
      if (newer) {
        pwm_time5 = micros();
      }
      else {
        pwm5 = micros() - pwm_time5;
      }
    }
  } // namespace radio_control
  
  /// \brief Monitor interrupts for new radio control PWM data and publish to ROS
  class RadioControlPwm {
    public:
      /// \brief Constructor
      ///
      /// \param topic : const char*
      ///       The ROS topic string to publish PWM data
      /// \param ch_pin0 : byte
      ///       The Arduino interrupt pin for PWM data on channel 0
      /// \param detect_pwm_isr0 : void (*detect_pwm_isr0)(void)
      ///       An interrupt service routine PWM data on channel 0
      /// Similarly for the remaining parameters... 
      RadioControlPwm(
        const char *topic,
        byte ch_pin0=CH_PIN0, void (*detect_pwm_isr0)(void)=radio_control::detect_pwm_isr0,
        byte ch_pin1=CH_PIN1, void (*detect_pwm_isr1)(void)=radio_control::detect_pwm_isr1,
        byte ch_pin2=CH_PIN2, void (*detect_pwm_isr2)(void)=radio_control::detect_pwm_isr2,
        byte ch_pin3=CH_PIN3, void (*detect_pwm_isr3)(void)=radio_control::detect_pwm_isr3,
        byte ch_pin4=CH_PIN4, void (*detect_pwm_isr4)(void)=radio_control::detect_pwm_isr4,
        byte ch_pin5=CH_PIN5, void (*detect_pwm_isr5)(void)=radio_control::detect_pwm_isr5
        ) :
        topic_(topic),
        ch_pin0_(ch_pin0),
        ch_pin1_(ch_pin1),
        ch_pin2_(ch_pin2),
        ch_pin3_(ch_pin3),
        ch_pin4_(ch_pin4),
        ch_pin5_(ch_pin5),
        detect_pwm_isr0_(detect_pwm_isr0),
        detect_pwm_isr1_(detect_pwm_isr1),
        detect_pwm_isr2_(detect_pwm_isr2),
        detect_pwm_isr3_(detect_pwm_isr3),
        detect_pwm_isr4_(detect_pwm_isr4),
        detect_pwm_isr5_(detect_pwm_isr5),
        channels_msg_(),
        channels_pub_(topic_, &channels_msg_)
      {
      }
  
      /// \brief Initialise the publishers and subscribers
      ///
      /// \param nh : ros::NodeHandle
      ///       The ROS node handle.
      void init(ros::NodeHandle &nh) {
        // Capture NodeHandle
        nh_ = &nh;
        
        // Advertise publishers
        nh_->advertise(channels_pub_);
  
        // Register interrupts
        enableInterrupt(ch_pin0_, detect_pwm_isr0_, CHANGE);
        enableInterrupt(ch_pin1_, detect_pwm_isr1_, CHANGE);
        enableInterrupt(ch_pin2_, detect_pwm_isr2_, CHANGE);
        enableInterrupt(ch_pin3_, detect_pwm_isr3_, CHANGE);
        enableInterrupt(ch_pin4_, detect_pwm_isr4_, CHANGE);
        enableInterrupt(ch_pin5_, detect_pwm_isr5_, CHANGE);
  
        // Initialise PWM values to mid-point
        for (uint8_t i=0; i<NUM_CHANNELS; ++i) {
          channels_[i] = 1500;
        }
      }
  
      /// \brief Update the PWM state with the latest values
      ///
      /// \param pwm0 The PWM state of channel0
      /// Similarly for the remaining parameters... 
      void update(
        int pwm0=radio_control::pwm0,
        int pwm1=radio_control::pwm1,
        int pwm2=radio_control::pwm2,
        int pwm3=radio_control::pwm3,
        int pwm4=radio_control::pwm4,
        int pwm5=radio_control::pwm5
      ) {
        // Capture the current pulse width modulation
        channels_[0] = pwm0;
        channels_[1] = pwm1;
        channels_[2] = pwm2;
        channels_[3] = pwm3;
        channels_[4] = pwm4;
        channels_[5] = pwm5;
        have_new_ = true;
      }
      
      /// \brief Publish all messages
      void publish() {
        channels_msg_.channels_length = channels_length_;
        channels_msg_.channels = channels_;
        channels_pub_.publish(&channels_msg_);
        have_new_ = false;
      }
      
      /// \brief Check if new data has been received from the RC receiver
      ///
      /// \returns True if new data has been received.
      bool have_new() const {
        return have_new_;
      }

    private:
      const char* topic_;
      byte ch_pin0_, ch_pin1_, ch_pin2_, ch_pin3_, ch_pin4_, ch_pin5_;
      void (*detect_pwm_isr0_)(void);
      void (*detect_pwm_isr1_)(void);
      void (*detect_pwm_isr2_)(void);
      void (*detect_pwm_isr3_)(void);
      void (*detect_pwm_isr4_)(void);
      void (*detect_pwm_isr5_)(void);
  
      curio_msgs::Channels channels_msg_;
      ros::Publisher channels_pub_;
      ros::NodeHandle* nh_ = 0;

      uint32_t channels_length_ = NUM_CHANNELS;
      uint16_t channels_[NUM_CHANNELS];    
      bool have_new_ = false;
  };

} // namespace curio_firmware

#endif // CURIO_FIRMWARE_RADIO_CONTROL_PWM_H_
