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

/*
 * Arduino firmware for Curio a Sawppy Rover.
 * 
 * The firmware includes:
 * 1. An interface to the Lewansoul Servo Bus Linker board for operating LX-16A servos
 * 2. A SUMD decoder for publishing 12 channel radio control signals  
 * 
 * Defines
 * -------
 * SERIAL_LX16A : HardwareSerial
 *    The hardware serial device used to communicate with the LX-16A servo bus, has (default Serial1)
 * SERIAL_RC: HardwareSerial
 *    The hardware serial device used to communicate with the RC receiver, has (default Serial2)
 * NUM_WHEELS : int
 *    The number of wheel servos, has (default 6)
 * NUM_STEERS : int
 *    The number of steering servos, has (default 4)
 * 
 * Parameters
 * ----------
 * ~wheel_servo_ids : list
 *    A list of integer wheel servo ids, has (default [11, 12, 13, 21, 22, 23]
 * ~steer_servo_ids : list
 *    A list of integer steering servo ids, has (default [111, 131, 211, 231]
 * ~steer_servo_angle_offsets : list
 *    A list of integer steering servo angle offsets for adjusting steering trim,
 *    has (default [0, 0, 0, 0]
 * 
 * Publications
 * ------------
 * servo/positions : curio_msgs::CurioServoPositions
 *    The wheel and steering servo positions (rate 30Hz)
 * servo/states : curio_msgs::CurioServoStates 
 *    The diagnostic state of each servo (rate 1Hz)
 * 
 * Subscriptions
 * -------------
 * servo/commands : curio_msgs::CurioServoCommands
 *    The commanded duty and position for the wheel and steering servos (expected rate 10Hz)
 * 
 * Graupner SUMD Decoder
 * ---------------------
 * Tested with a Graupner mz-12 PRO transmitter and Falcon 12 2.4 GHz receiver.
 * 
 * The Falcon 12 has 6 servo channels. Channel 5 may be used for a servo, sensor, voltage or
 * as a SUMD channel. SUMD is a serial UART protocol (see HoTT SUMD Data Protocol below).
 * Configuring the transmitter/receiver to use SUMD gives us 12 usable channels instead of six.
 * 
 * The code for the SUMD decoder was sourced from Ardupilot and modified for the Arduino
 * to use a standalone CRC calculation from the HoTT technical documents. The Ardupilot
 * CRC code is GPL licensed so we cannot use it in this project. The SUMD code itself
 * is licensed under BSD-3-Clause and copyright (c) 2015 PX4 Development Team.
 * 
 * Resources
 * ---------
 * 
 * HoTT SUMD Data Protocol
 * https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
 *       
 * ArduPilot SUMD decoder
 * https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/utility/sumd.h
 * https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/utility/sumd.h
 *    
 * Kinetis Drone: Remote Controller with SUMD
 * https://mcuoneclipse.com/2015/11/01/kinetis-drone-remote-controller-with-sumd/
 *
 */

#define RADIO_CONTROL_ENABLE  1
#define RADIO_CONTROL_SUMD    1

#include "ros.h"
#include "lobot_serial.h"

#if RADIO_CONTROL_ENABLE
#if RADIO_CONTROL_SUMD
#include "radio_control_sumd.h"
#else
#include "radio_control_pwm.h"
#endif
#endif

#include <curio_msgs/CurioServoCommands.h>
#include <curio_msgs/CurioServoPositions.h>
#include <curio_msgs/CurioServoStates.h>
#include <curio_msgs/LX16AState.h>

#include <HardwareSerial.h>
#include <inttypes.h>

#define SERIAL_LX16A  Serial1

#if RADIO_CONTROL_ENABLE
#define SERIAL_RC     Serial2
#endif

#define NUM_WHEELS    6
#define NUM_STEERS    4

// ROS node
ros::NodeHandle nh;

////////////////////////////////////////////////////////////////////////////////
// LX-16A Servo 

// CurioServoPositions (servo/positions) - lightweight messages with fast update (~ 25 Hz)
curio_msgs::CurioServoPositions curio_positions_msg_;
ros::Publisher curio_positions_pub_("servo/positions", &curio_positions_msg_);
int16_t wheel_positions_[NUM_WHEELS];
int16_t steer_positions_[NUM_STEERS]; 

// Default servo identifiers and angle offsets, overwritten by param server values if available.
uint16_t wheel_ids_[NUM_WHEELS] = {11, 12, 13, 21, 22, 23};
uint16_t steer_ids_[NUM_STEERS] = {111, 131, 211, 231};
uint16_t steer_angle_offsets_[NUM_STEERS] = {0, 0, 0, 0};

// CurioServoStates (servo/states) - updated at a slower rate (~ 1 Hz)
curio_msgs::CurioServoStates curio_states_msg_;
ros::Publisher curio_states_pub_("servo/states", &curio_states_msg_);
curio_msgs::LX16AState wheel_states_[NUM_WHEELS]; 
curio_msgs::LX16AState steer_states_[NUM_STEERS]; 

// CurioServoCommands (servo/commands)
#define SERVO_COMMAND_TIMEOUT 750   // [ms]
uint32_t last_cmd_rec_millis = 0;   // [ms]
int16_t wheel_commands_[NUM_WHEELS];
int16_t steer_commands_[NUM_STEERS]; 

void servoCommandsCallback(const curio_msgs::CurioServoCommands& msg) {
  // Update watchdog for servo commands  
  last_cmd_rec_millis = millis();
  
  // Update wheel and steering servo commands.  
  for (uint8_t i=0; i<NUM_WHEELS; ++i) {
    wheel_commands_[i] = msg.wheel_commands[i];
  }

  for (uint8_t i=0; i<NUM_STEERS; ++i) {
    steer_commands_[i] = msg.steer_commands[i];
  }
}

ros::Subscriber<curio_msgs::CurioServoCommands> curio_commands_sub_("servo/commands", &servoCommandsCallback);

#define READ_POS_UPDATE_RATE  30    // [Hz]
uint32_t last_pos_millis     = 0;   // [ms]

#define CONTROL_UPDATE_RATE   30    // [Hz]
uint32_t last_control_millis = 0;   // [ms]

// Stagger the updates for diagnostics
#define READ_VIN_UPDATE_RATE   1    // [Hz]
uint32_t last_vin_millis     = 0;   // [ms]

#define READ_TEMP_UPDATE_RATE  1    // [Hz]
uint32_t last_temp_millis  = 200;   // [ms]

#define READ_TRIM_UPDATE_RATE  1    // [Hz]
uint32_t last_trim_millis  = 400;   // [ms]

#define PUB_STAT_UPDATE_RATE   1    // [Hz]
uint32_t last_stat_millis  = 600;   // [ms]

////////////////////////////////////////////////////////////////////////////////
// Radio Control SUMD Decoder

#if RADIO_CONTROL_ENABLE

#define RC_UPDATE_RATE        20    // [Hz]
uint32_t last_rc_millis      = 0;   // [ms]

#if RADIO_CONTROL_SUMD
curio_firmware::RadioControlSumD rc(SERIAL_RC, "radio_ctrl/channels");
#else
curio_firmware::RadioControlPwm rc("radio_ctrl/channels");
#endif

#endif


void setup() {  
  // Initialise serial communication
  SERIAL_LX16A.begin(115200);

#if RADIO_CONTROL_ENABLE
  SERIAL_RC.begin(115200);
#endif

  // Initialise the ROS node
  nh.initNode();

  // Wait for connection
  while(!nh.connected()) {
    nh.spinOnce();
  }
  nh.loginfo("Starting Curio Arduino node...");
    
  // Initialise ROS interfaces for RC
#if RADIO_CONTROL_ENABLE
  nh.loginfo("Initialising radio control decoder");  
  rc.init(nh);
#endif

  // Parameters
  if (nh.getParam("~wheel_servo_ids", wheel_ids_, NUM_WHEELS)) {
    nh.loginfo("Loaded params: wheel_servo_ids");
  }
  if (nh.getParam("~steer_servo_ids", steer_ids_, NUM_STEERS)) {
    nh.loginfo("Loaded params: steer_servo_ids");
  }
  if (nh.getParam("~steer_servo_angle_offsets", steer_angle_offsets_, NUM_STEERS)) {
    nh.loginfo("Loaded params: steer_servo_angle_offsets");
  }

  // Publications
  nh.advertise(curio_positions_pub_);
  nh.advertise(curio_states_pub_);

  // Subscriptions
  nh.subscribe(curio_commands_sub_);

  // Set steering trim and servo mode.
  nh.loginfo("Setting steering trim");
  for (uint8_t i=0; i<NUM_STEERS; ++i) {
    uint8_t servo_id = steer_ids_[i];
    int16_t deviation = steer_angle_offsets_[i];    
    LobotSerialServoAngleAdjust(SERIAL_LX16A, servo_id, deviation);
    LobotSerialServoSetMode(SERIAL_LX16A, servo_id, 0, 0);
  }

  // Initialise the commanded wheel duty and steering position  
  nh.loginfo("Initialising wheel duty and steering position");
  for (uint8_t i=0; i<NUM_WHEELS; ++i) {
    wheel_commands_[i] = 0;
  }
  for (uint8_t i=0; i<NUM_STEERS; ++i) {
    steer_commands_[i] = 500;
  }

  nh.spinOnce();  
}

void loop() {
#if RADIO_CONTROL_ENABLE
  // Process RC data.
  rc.update();
  
  // Publish the most recently decoded message.
  if (millis() > last_rc_millis + 1000/RC_UPDATE_RATE) {
    last_rc_millis += 1000/RC_UPDATE_RATE;
    if (rc.have_new()) {
      rc.publish();
      nh.spinOnce();
    }
  }
#endif

  // Read (and publish) the servo positions
  if (millis() > last_pos_millis + 1000/READ_POS_UPDATE_RATE) {    
    last_pos_millis += 1000/READ_POS_UPDATE_RATE;

    // Read wheel servos
    for (uint8_t i=0; i<NUM_WHEELS; ++i) {
      uint8_t servo_id = wheel_ids_[i];
      int32_t pos = LobotSerialServoReadPosition(SERIAL_LX16A, servo_id);
      if (pos != -1) {
        wheel_positions_[i] = pos;
      }
    }

    // Read steer servos
    for (uint8_t i=0; i<NUM_STEERS; ++i) {
      uint8_t servo_id = steer_ids_[i];
      int32_t pos = LobotSerialServoReadPosition(SERIAL_LX16A, servo_id);
      if (pos != -1) {
        steer_positions_[i] = pos;
      }
    }
        
    // Publish servo positions
    curio_positions_msg_.header.stamp = nh.now();
    curio_positions_msg_.header.frame_id = "base_link";
    curio_positions_msg_.wheel_positions_length = NUM_WHEELS;
    curio_positions_msg_.wheel_positions = wheel_positions_;
    curio_positions_msg_.steer_positions_length = NUM_STEERS;
    curio_positions_msg_.steer_positions = steer_positions_;
    curio_positions_pub_.publish(&curio_positions_msg_);

    nh.spinOnce();
  }    

  // Send move commands to the wheel and steering servos
  if (millis() > last_control_millis + 1000/CONTROL_UPDATE_RATE) {    
    last_control_millis += 1000/CONTROL_UPDATE_RATE;

    // Check whether we have had a servo command recently. If so we stop the motors.
    bool has_timed_out = millis() > last_cmd_rec_millis + SERVO_COMMAND_TIMEOUT;

    // Set the duty for the wheel servos
    for (uint8_t i=0; i<NUM_WHEELS; ++i) {
      uint8_t servo_id = wheel_ids_[i];
      int16_t duty = has_timed_out ? 0 : wheel_commands_[i];
      LobotSerialServoSetMode(SERIAL_LX16A, servo_id, 1, duty);
    }
  
    // Set the position of the steer servos    
    for (uint8_t i=0; i<NUM_STEERS; ++i) {
      uint8_t servo_id = steer_ids_[i];
      int16_t position = steer_commands_[i];
      LobotSerialServoMove(SERIAL_LX16A, servo_id, position, 50);
    }

    nh.spinOnce();
  }

  // Read servo voltages
  if (millis() > last_vin_millis + 1000/READ_VIN_UPDATE_RATE) {
    last_vin_millis += 1000/READ_VIN_UPDATE_RATE;

    // Read wheel servo vin
    for (uint8_t i=0; i<NUM_WHEELS; ++i) {
      uint8_t servo_id = wheel_ids_[i];
      int16_t vin = LobotSerialServoReadVin(SERIAL_LX16A, servo_id);
      if (vin != -2048) {
        wheel_states_[i].voltage = vin/1000.0;
      }
    }

      // Read steer servo vin
    for (uint8_t i=0; i<NUM_STEERS; ++i) {
      uint8_t servo_id = steer_ids_[i];
      int16_t vin = LobotSerialServoReadVin(SERIAL_LX16A, servo_id);    
      if (vin != -2048) {
        steer_states_[i].voltage = vin/1000.0;
      }
    }

    nh.spinOnce();
  }

  // Publish servo status 
  if (millis() > last_stat_millis + 1000/PUB_STAT_UPDATE_RATE) {
    last_stat_millis += 1000/PUB_STAT_UPDATE_RATE;

    // @TODO: move to setup
    for (uint8_t i=0; i<NUM_WHEELS; ++i) {
      uint8_t servo_id = wheel_ids_[i];
      wheel_states_[i].id = servo_id;
      wheel_states_[i].mode = 1;
      wheel_states_[i].angle_offset = 0;
      wheel_states_[i].position = wheel_positions_[i];
    }

    // @TODO: move to setup
    for (uint8_t i=0; i<NUM_STEERS; ++i) {
      uint8_t servo_id = steer_ids_[i];
      steer_states_[i].id = servo_id;
      steer_states_[i].mode = 0;
      steer_states_[i].angle_offset = 0;
      steer_states_[i].position = steer_positions_[i];
    }

    // Update overall state
    curio_states_msg_.header.stamp = nh.now();
    curio_states_msg_.header.frame_id = "base_link";
    curio_states_msg_.wheel_states_length = NUM_WHEELS;
    curio_states_msg_.wheel_states = wheel_states_;
    curio_states_msg_.steer_states_length = NUM_STEERS;
    curio_states_msg_.steer_states = steer_states_;
    curio_states_pub_.publish(&curio_states_msg_);

    nh.spinOnce();
  }
    
  nh.spinOnce();  
}
