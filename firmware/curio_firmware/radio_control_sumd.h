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

#ifndef CURIO_FIRMWARE_RADIO_CONTROL_SUMD_H_
#define CURIO_FIRMWARE_RADIO_CONTROL_SUMD_H_

#include "ros.h"
#include "sumd.h"

#include <curio_msgs/Channels.h>
#include <HardwareSerial.h>

// Identify the start of the SUMD message
#define SUMD_HEADER_VENDOR_ID 0xA8  

// The number of channels to decode
#define NUM_CHANNELS          12

namespace curio_firmware {
  
  /// \brief Decode radio control SUMD messages and publish to ROS
  class RadioControlSumD {
    public:  
      
      /// \brief Constructor
      ///
      /// \param serial : HardwareSerial
      ///       A reference to the hardware serial device
      /// \param topic : const char*
      ///       The ROS topic string to publish PWM data
      RadioControlSumD(HardwareSerial& serial, const char *topic) :
        serial_(serial),
        topic_(topic),
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
      }

      /// \brief Publish all messages
      void publish() {
        channels_msg_.channels_length = channels_length_;
        channels_msg_.channels = (int16_t*)channels_;
        channels_pub_.publish(&channels_msg_);
        have_new_ = false;
      }
    
      /// \brief Check serial for messages and decode 
      void update() {
        uint16_t num_available = 0; 
        uint16_t num_bytes = 0; 
        while ((num_available = serial_.available()) > 0) {
      
          uint8_t in_byte = serial_.read();
          num_bytes++;
      
          // Monitor potential issue with overflow:
          // The serial buffer holds 64 bytes and the SUMD message
          // contains 37 bytes. If we read more than 27 bytes before
          // encountering SUMD_HEADER_VENDOR_ID (0xA8), and the buffer
          // is full, then we can't complete the message, because the
          // additional data will be discarded - in this case flush
          // the buffer and break.
          bool has_overflowed = num_available > 63;    
          if (in_byte == SUMD_HEADER_VENDOR_ID) {
            // Reset the buffer position      
            if (has_overflowed && num_bytes > 26) {
              flushSerialRx();
              break;
            }
          }
      
          int res = sumd_decode(in_byte, &rssi_, &rx_count_, &channel_count_, channels_, channels_length_);
          switch (res) {
          case 0:   // Success
            have_new_ = true;
            return;     
          case 1:   // Accumulating - continue
            // Buffer the current message.
            break;
          case 2:   // Unknown packet
            return;
          case 3:   // Out of sync - continue
            break;
          case 4:   // Checksum error
            return;
          default:  // Unknown error
            return;
          }      
        }
      }
  
      /// \brief Check if new data has been received from the RC receiver
      ///
      /// \returns True if new data has been received and decoded.
      bool have_new() const {
        return have_new_;
      }
      
    private:
      HardwareSerial& serial_;
      const char* topic_;
      curio_msgs::Channels channels_msg_;
      ros::Publisher channels_pub_;
      ros::NodeHandle* nh_ = 0;    
  
      uint32_t channels_length_ = NUM_CHANNELS;
      uint16_t channels_[NUM_CHANNELS];    
      bool have_new_ = false;
      
      // SUMD decoder workspace variables
      uint8_t rssi_ = 0;
      uint8_t rx_count_ = 0;
      uint16_t channel_count_ = 0;

      /// \brief Flush the serial input buffer.
      void flushSerialRx() {
        while (serial_.read() > 0);
      }
    
  };

} // namespace curio_firmware

#endif // CURIO_FIRMWARE_RADIO_CONTROL_SUMD_H_
