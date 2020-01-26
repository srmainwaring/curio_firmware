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

#ifndef CURIO_FIRMWARE_RADIO_CONTROL_H
#define CURIO_FIRMWARE_RADIO_CONTROL_H

#include "ros.h"

#include <curio_msgs/Channels.h>

#define NUM_CHANNELS 12

class RadioControl {
  public:  
    uint32_t channels_length = NUM_CHANNELS;
    uint16_t channels[NUM_CHANNELS];    
    
    RadioControl(const char *topic) :
      topic_(topic),
      channels_msg_(),
      channels_pub_(topic_, &channels_msg_)
    {
    }

    void init(ros::NodeHandle &nh) {
      // Capture NodeHandle
      nh_ = &nh;
      
      // Advertise publishers
      nh_->advertise(channels_pub_);
    }
    
    void publish() {
      channels_msg_.channels_length = channels_length;
      channels_msg_.channels = channels;
      channels_pub_.publish(&channels_msg_);
    }

  private:
    const char* topic_;
    curio_msgs::Channels channels_msg_;
    ros::Publisher channels_pub_;
    ros::NodeHandle* nh_ = 0;    
};

#endif // CURIO_FIRMWARE_RADIO_CONTROL_H
