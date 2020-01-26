#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#if ARDUINO>=100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <HardwareSerial.h>
#define SERIAL_CLASS HardwareSerial

/*
 * Standard baudrates: 57600, 74800, 115200, 230400, 250000, 500000
 */
class ArduinoHardware
{
  public:
    ArduinoHardware()
    {
      iostream = &Serial;
      baud_ = 115200;
    }
  
    void setBaud(long baud)
    {
      this->baud_= baud;
    }
  
    int getBaud()
    {
      return baud_;
    }

    void init()
    {
      iostream->begin(baud_);
    }

    int read()
    {
      return iostream->read();
    };

    void write(uint8_t* data, int length)
    {
      for(int i=0; i<length; i++)
      {
        iostream->write(data[i]);
      }
    }

    unsigned long time()
    {
      return millis();
    }

  protected:
    SERIAL_CLASS* iostream;
    long baud_;
};

#endif // ROS_ARDUINO_HARDWARE_H_
