/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: July 18, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a library of functions for the RPi to communicate with the BrickPi.
*/

#ifndef __BrickPi_h_
#define __BrickPi_h_

#define DEBUG

#include <stdlib.h>
#include <signal.h>
#include <wiringPi.h>

#ifndef Max
#define Max(x, y) (x>y?x:y)                 // Return the highest value
#endif
#ifndef Min
#define Min(x, y) (x<y?x:y)                 // Return the lowest value
#endif
#ifndef Clip
#define Clip(v, x, y) (Min(Max(v, x), y))   // Return the value, clipped to x <= v <= y
#endif
#ifndef Dead
#define Dead(v, x, y) ((v<=x)?v:(v>=y?v:0)) // Return the value, with a dead spot from x to y
#endif
#ifndef Abs
#define Abs(v) (v<0?(-v):v)                 // Return the absolute value - untested
#endif

#define LED_1   0
#define LED_2   1
#define LED_ON  1023
#define LED_OFF 0

#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define PORT_D 3

#define TYPE_MOTOR_FLOAT     0    // Float motor
#define TYPE_MOTOR_SPEED     1    // Motor speed control
#define TYPE_MOTOR_POSITION  2    // Motor position control
  #define MOTOR_KP_DEFAULT   2.0  // Motor position control - Proportional Konstant
  #define MOTOR_KD_DEFAULT   5.0  // Motor position control - Derivative Konstant
  #define MOTOR_DEAD_DEFAULT 10   // A dead-spot in the active motor control. For speeds in the range of -MOTOR_DEAD_DEFAULT to MOTOR_DEAD_DEFAULT, the motor won't run. Outside that range, the motor value is then increased (from 0) by MOTOR_DEAD_DEFAULT.

#define PORT_1 0
#define PORT_2 1
#define PORT_3 2
#define PORT_4 3

#define MASK_D0_M 0x01
#define MASK_D1_M 0x02
#define MASK_9V   0x04
#define MASK_D0_S 0x08
#define MASK_D1_S 0x10

#define BYTE_MSG_TYPE               0 // MSG_TYPE is the first byte.
  #define MSG_TYPE_CHANGE_ADDR      1 // Change the UART address.
  #define MSG_TYPE_SENSOR_TYPE      2 // Change/set the sensor type.
  #define MSG_TYPE_VALUES           3 // Set the motor speed and direction, and return the sesnors and encoders.
  #define MSG_TYPE_E_STOP           4 // Float motors immidately
  #define MSG_TYPE_TIMEOUT_SETTINGS 5 // Set the timeout

  // New UART address (MSG_TYPE_CHANGE_ADDR)
    #define BYTE_NEW_ADDRESS     1
  
  // Sensor setup (MSG_TYPE_SENSOR_TYPE)
    #define BYTE_SENSOR_1_TYPE   1
    #define BYTE_SENSOR_2_TYPE   2
  
  // Timeout setup (MSG_TYPE_TIMEOUT_SETTINGS)
    #define BYTE_TIMEOUT 1

#define TYPE_SENSOR_RAW                0 // - 31
#define TYPE_SENSOR_LIGHT_OFF          0
#define TYPE_SENSOR_LIGHT_ON           (MASK_D0_M | MASK_D0_S)
#define TYPE_SENSOR_TOUCH              32
#define TYPE_SENSOR_ULTRASONIC_CONT    33
#define TYPE_SENSOR_ULTRASONIC_SS      34
#define TYPE_SENSOR_RCX_LIGHT          35 // tested minimally
#define TYPE_SENSOR_COLOR_FULL         36
#define TYPE_SENSOR_COLOR_RED          37
#define TYPE_SENSOR_COLOR_GREEN        38
#define TYPE_SENSOR_COLOR_BLUE         39
#define TYPE_SENSOR_COLOR_NONE         40
#define TYPE_SENSOR_I2C                41
#define TYPE_SENSOR_I2C_9V             42

#define BIT_I2C_MID  0x01  // Do one of those funny clock pulses between writing and reading. defined for each device.
#define BIT_I2C_SAME 0x02  // The transmit data, and the number of bytes to read and write isn't going to change. defined for each device.

#define INDEX_RED   0
#define INDEX_GREEN 1
#define INDEX_BLUE  2
#define INDEX_BLANK 3

int BrickPiSetLed(unsigned char led, int value);
void BrickPiUpdateLEDs(void);
void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]);

struct BrickPiStruct{
/*
  LEDs
*/
  int LED                              [2];        // The state of the two LEDs

  unsigned char Address                [2];        // Communication addresses
  unsigned long Timeout                   ;        // Communication timeout (how long in ms since the last valid communication before floating the motors). 0 disables the timeout.

/*
  Motors
*/
  int           MotorSpeed             [4];        // Motor speeds, from -255 to 255
  unsigned char MotorEnable            [4];        // Motor mode. Float, Speed, Position.
  long          MotorTarget            [4];        // Motor target position. This is implemented on the RPi, not in the BrickPi FW.
  long          MotorTargetLastError   [4];        // Value used internally for motor position regulation.
  float         MotorTargetKP          [4];        // Percent Konstant - used for motor position regulation.
  float         MotorTargetKD          [4];        // Derivative Konstant - used for motor position regulation.
  unsigned char MotorDead              [4];        // How wide of a gap to leave between 0 and the value speed value used for running to a target position.

/*
  Encoders
*/
  long          EncoderOffset          [4];        // Encoder offsets
  long          Encoder                [4];        // Encoder values

/*
  Sensors
*/
  long          Sensor                 [4];        // Primary sensor values
  long          SensorArray            [4][4];     // For more sensor values for the sensor (e.g. for color sensor FULL mode).
  unsigned char SensorType             [4];        // Sensor types
  unsigned char SensorSettings         [4][8];     // Sensor settings, used for specifying I2C settings.

/*
  I2C
*/
  unsigned char SensorI2CDevices       [4];        // How many I2C devices are on each bus (1 - 8).
  unsigned char SensorI2CSpeed         [4];        // The I2C speed.
  unsigned char SensorI2CAddr          [4][8];     // The I2C address of each device on each bus.  
  unsigned char SensorI2CWrite         [4][8];     // How many bytes to write
  unsigned char SensorI2CRead          [4][8];     // How many bytes to read
  unsigned char SensorI2COut           [4][8][16]; // The I2C bytes to write
  unsigned char SensorI2CIn            [4][8][16]; // The I2C input buffers
};

struct BrickPiStruct BrickPi;

unsigned char Array[256];
unsigned char BytesReceived;

int BrickPiEmergencyStop(){
/*
  Try 3 times to send E Stop to each of the uCs.
  If failed:
    Broadcast E Stop 3 times.
*/
  
  unsigned char i = 0;
  while(i < 3){
    unsigned char ii = 0;
    while(ii < 2){
      Array[BYTE_MSG_TYPE] = MSG_TYPE_E_STOP;
      BrickPiTx(BrickPi.Address[ii], 1, Array);
      if(BrickPiRx(&BytesReceived, Array, 2500)){
        goto NEXT_TRY;
      }
      if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_E_STOP)){
        goto NEXT_TRY;
      }
      if(ii == 1){
        return 0;
      }
      ii++;
    }
NEXT_TRY:
    i++;
  }
  
  i = 0;
  while(i < 3){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_E_STOP;
    BrickPiTx(0, 1, Array);
    usleep(5000);
    i++;
  }
  return -1;
}

int BrickPiChangeAddress(unsigned char OldAddr, unsigned char NewAddr){
//  unsigned char i = 0;
  Array[BYTE_MSG_TYPE] = MSG_TYPE_CHANGE_ADDR;
  Array[BYTE_NEW_ADDRESS] = NewAddr;
  BrickPiTx(OldAddr, 2, Array);
  
  if(BrickPiRx(&BytesReceived, Array, 5000))
    return -1;
  if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_CHANGE_ADDR))
    return -1;
  
  return 0;
}

int BrickPiSetTimeout(){
  unsigned char i = 0;
  while(i < 2){
    Array[BYTE_MSG_TYPE] = MSG_TYPE_TIMEOUT_SETTINGS;
    Array[ BYTE_TIMEOUT     ] = ( BrickPi.Timeout             & 0xFF);
    Array[(BYTE_TIMEOUT + 1)] = ((BrickPi.Timeout / 256     ) & 0xFF);
    Array[(BYTE_TIMEOUT + 2)] = ((BrickPi.Timeout / 65536   ) & 0xFF);
    Array[(BYTE_TIMEOUT + 3)] = ((BrickPi.Timeout / 16777216) & 0xFF);
    BrickPiTx(BrickPi.Address[i], 5, Array);
    if(BrickPiRx(&BytesReceived, Array, 2500))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_TIMEOUT_SETTINGS))
      return -1;
    i++;
  }
  return 0;
}

unsigned int Bit_Offset = 0;

void AddBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits, unsigned long value){
  unsigned char i = 0;
  while(i < bits){
    if(value & 0x01){
      Array[(byte_offset + ((bit_offset + Bit_Offset + i) / 8))] |= (0x01 << ((bit_offset + Bit_Offset + i) % 8));
    }
    value /= 2;
    i++;
  }
  Bit_Offset += bits;
}

unsigned long GetBits(unsigned char byte_offset, unsigned char bit_offset, unsigned char bits){
  unsigned long Result = 0;
  char i = bits;
  while(i){
    Result *= 2;
    Result |= ((Array[(byte_offset + ((bit_offset + Bit_Offset + (i - 1)) / 8))] >> ((bit_offset + Bit_Offset + (i - 1)) % 8)) & 0x01);    
    i--;
  }
  Bit_Offset += bits;
  return Result;
}

unsigned char BitsNeeded(unsigned long value){
  unsigned char i = 0;
  while(i < 32){
    if(!value)
      return i;
    value /= 2;
    i++;
  }
  return 31;
}

int BrickPiSetupSensors(){
  unsigned char i = 0;
  while(i < 2){
    int ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    Bit_Offset = 0;
    Array[BYTE_MSG_TYPE] = MSG_TYPE_SENSOR_TYPE;
    Array[BYTE_SENSOR_1_TYPE] = BrickPi.SensorType[PORT_1 + (i * 2)];
    Array[BYTE_SENSOR_2_TYPE] = BrickPi.SensorType[PORT_2 + (i * 2)];
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C
      || Array[BYTE_SENSOR_1_TYPE + ii] == TYPE_SENSOR_I2C_9V){
        AddBits(3, 0, 8, BrickPi.SensorI2CSpeed[port]);
        
        if(BrickPi.SensorI2CDevices[port] > 8)
          BrickPi.SensorI2CDevices[port] = 8;
        
        if(BrickPi.SensorI2CDevices[port] == 0)
          BrickPi.SensorI2CDevices[port] = 1;
        
        AddBits(3, 0, 3, (BrickPi.SensorI2CDevices[port] - 1));
        
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          AddBits(3, 0, 7, (BrickPi.SensorI2CAddr[port][device] >> 1));
          AddBits(3, 0, 2, BrickPi.SensorSettings[port][device]);
          if(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME){          
            AddBits(3, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(3, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(3, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 3);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    if(BrickPiRx(&BytesReceived, Array, 1000000))
      return -1;
    if(!(BytesReceived == 1 && Array[BYTE_MSG_TYPE] == MSG_TYPE_SENSOR_TYPE))
      return -1;
    i++;
  }
  return 0;
}

unsigned char Retried = 0; // For re-trying a failed update.

int BrickPiUpdateValues(){
  BrickPiUpdateLEDs();
  
  unsigned char i = 0;
  unsigned int ii = 0;
  while(i < 2){
    Retried = 0;    

__RETRY_COMMUNICATION__:
    
    ii = 0;
    while(ii < 256){
      Array[ii] = 0;
      ii++;
    }
    
    Array[BYTE_MSG_TYPE] = MSG_TYPE_VALUES;
    
    Bit_Offset = 0;
    
//    AddBits(1, 0, 2, 0);     use this to disable encoder offset
    
    ii = 0;                 // use this for encoder offset support
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.EncoderOffset[port]){
        long Temp_Value = BrickPi.EncoderOffset[port];
        unsigned char Temp_ENC_DIR = 0;
        unsigned char Temp_BitsNeeded = 0;
        
        AddBits(1, 0, 1, 1);
        if(Temp_Value < 0){
          Temp_ENC_DIR = 1;
          Temp_Value *= (-1);
        }        
        Temp_BitsNeeded = BitsNeeded(Temp_Value);
        AddBits(1, 0, 5, Temp_BitsNeeded);
        Temp_BitsNeeded++;
        Temp_Value *= 2;
        Temp_Value |= Temp_ENC_DIR;
        AddBits(1, 0, Temp_BitsNeeded, Temp_Value);
      }
      else{
        AddBits(1, 0, 1, 0);
      }
      ii++;
    }
    
    int speed;
    unsigned char dir;    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      
      if(BrickPi.MotorEnable[port] == TYPE_MOTOR_FLOAT){
        AddBits(1, 0, 10, 0);
      }else{
        if(BrickPi.MotorEnable[port] == TYPE_MOTOR_SPEED){
          speed = BrickPi.MotorSpeed[port];
        }else if(BrickPi.MotorEnable[port] == TYPE_MOTOR_POSITION){
          long error = BrickPi.MotorTarget[port] - BrickPi.Encoder[port];
          float speed_f = (error * BrickPi.MotorTargetKP[port]) + ((error - BrickPi.MotorTargetLastError[port]) * BrickPi.MotorTargetKD[port]);
          BrickPi.MotorTargetLastError[port] = error;
          if(speed_f < BrickPi.MotorDead[port] && speed_f > -BrickPi.MotorDead[port]){
            speed_f = 0;
          }
          if(speed_f > 0){
            speed_f += BrickPi.MotorDead[port];
          }else if(speed_f < 0){
            speed_f -= BrickPi.MotorDead[port];
          }
          speed = Clip(speed_f, -255, 255); // Clip the speed to the range of -255 to 255.
/*#ifdef DEBUG
          printf("Speed: %d\n", speed);        
#endif*/
        }
        
        dir = 0;
        if(speed < 0){
          dir = 1;
          speed *= (-1);
        }
        if(speed > 255){
          speed = 255;
        }
        AddBits(1, 0, 10, ((((speed & 0xFF) << 2) | (dir << 1) | (0x01)) & 0x3FF));
      }
      ii++;
    }
    
    ii = 0;
    while(ii < 2){
      unsigned char port = (i * 2) + ii;
      if(BrickPi.SensorType[port] == TYPE_SENSOR_I2C
      || BrickPi.SensorType[port] == TYPE_SENSOR_I2C_9V){
        unsigned char device = 0;
        while(device < BrickPi.SensorI2CDevices[port]){
          if(!(BrickPi.SensorSettings[port][device] & BIT_I2C_SAME)){
            AddBits(1, 0, 4, BrickPi.SensorI2CWrite[port][device]);
            AddBits(1, 0, 4, BrickPi.SensorI2CRead [port][device]);
            unsigned char out_byte = 0;
            while(out_byte < BrickPi.SensorI2CWrite[port][device]){
              AddBits(1, 0, 8, BrickPi.SensorI2COut[port][device][out_byte]);
              out_byte++;
            }
          }
          device++;
        }
      }
      ii++;
    }
    
    unsigned char UART_TX_BYTES = (((Bit_Offset + 7) / 8) + 1);
    BrickPiTx(BrickPi.Address[i], UART_TX_BYTES, Array);
    usleep(500);
    int result = BrickPiRx(&BytesReceived, Array, 10000);
    
    if(result != -2){                            // -2 is the only error that indicates that the BrickPi uC did not properly receive the message
      BrickPi.EncoderOffset[((i * 2) + PORT_A)] = 0;
      BrickPi.EncoderOffset[((i * 2) + PORT_B)] = 0;
    }
    
    if(result || (Array[BYTE_MSG_TYPE] != MSG_TYPE_VALUES)){
#ifdef DEBUG
      printf("BrickPiRx error: %d\n", result);
#endif
      if(Retried < 4){
        Retried++;
        goto __RETRY_COMMUNICATION__;
      }
      else{
#ifdef DEBUG
        printf("Retry failed.\n");
#endif
        return -1;
      }      
    }
    
    Bit_Offset = 0;
    
    unsigned char Temp_BitsUsed[2] = {0, 0};         // Used for encoder values
    Temp_BitsUsed[0] = GetBits(1, 0, 5);
    Temp_BitsUsed[1] = GetBits(1, 0, 5);
    unsigned long Temp_EncoderVal;
    
    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      Temp_EncoderVal = GetBits(1, 0, Temp_BitsUsed[ii]);
      if(Temp_EncoderVal & 0x01){
        Temp_EncoderVal /= 2;
        BrickPi.Encoder[port] = Temp_EncoderVal * (-1);}
      else{
        BrickPi.Encoder[port] = (Temp_EncoderVal / 2);}
      ii++;
    }

    ii = 0;
    while(ii < 2){
      unsigned char port = ii + (i * 2);
      switch(BrickPi.SensorType[port]){
        case TYPE_SENSOR_TOUCH:
          BrickPi.Sensor[port] = GetBits(1, 0, 1);
        break;
        case TYPE_SENSOR_ULTRASONIC_CONT:
        case TYPE_SENSOR_ULTRASONIC_SS:
          BrickPi.Sensor[port] = GetBits(1, 0, 8);
        break;
        case TYPE_SENSOR_COLOR_FULL:
          BrickPi.Sensor[port] = GetBits(1, 0, 3);
          BrickPi.SensorArray[port][INDEX_BLANK] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_RED  ] = GetBits(1, 0, 10);                
          BrickPi.SensorArray[port][INDEX_GREEN] = GetBits(1, 0, 10);
          BrickPi.SensorArray[port][INDEX_BLUE ] = GetBits(1, 0, 10);
        break;          
        case TYPE_SENSOR_I2C:
        case TYPE_SENSOR_I2C_9V:
          BrickPi.Sensor[port] = GetBits(1, 0, BrickPi.SensorI2CDevices[port]);
          unsigned char device = 0;
          while(device < BrickPi.SensorI2CDevices[port]){
            if(BrickPi.Sensor[port] & (0x01 << device)){
              unsigned char in_byte = 0;
              while(in_byte < BrickPi.SensorI2CRead[port][device]){
                BrickPi.SensorI2CIn[port][device][in_byte] = GetBits(1, 0, 8);
                in_byte++;
              }
            }
            device++;
          }
        break;      
        case TYPE_SENSOR_LIGHT_OFF:
        case TYPE_SENSOR_LIGHT_ON:
        case TYPE_SENSOR_RCX_LIGHT:
        case TYPE_SENSOR_COLOR_RED:
        case TYPE_SENSOR_COLOR_GREEN:
        case TYPE_SENSOR_COLOR_BLUE:
        case TYPE_SENSOR_COLOR_NONE:
        default:
          BrickPi.Sensor[(ii + (i * 2))] = GetBits(1, 0, 10);
      }        
      ii++;
    }      
    i++;
  }
  return 0;
}

int BrickPiSetLed(unsigned char led, int value){
  switch(led){
    case LED_1:
      pwmWrite    (1,  value     );    // Set the PWM of LED 1 (0-1023)
    break;
    case LED_2:
      digitalWrite(2, (value?1:0));    // Set the state of LED 2
    break;
    default:
      return -1;
  }
  BrickPi.LED[led] = value;
  return 0;
}

void BrickPiUpdateLEDs(){
  pwmWrite    (1,  BrickPi.LED[LED_1]     );     // Set the PWM of LED 1 (0-1023)
  digitalWrite(2, (BrickPi.LED[LED_2]?1:0));     // Set the state of LED 2
}

int UART_file_descriptor = 0; 

/*
  To safely shutdown the program, use:
    sudo killall program -s 2
  which sends signal 2 to process "program"
*/

void BrickPiExitSafely(int sig)                  // Exit the program safely
{
  signal(SIGINT , SIG_IGN);                      // Disable the signal interrupt
  signal(SIGQUIT, SIG_IGN);                      // Disable the signal interrupt
#ifdef DEBUG
  printf("\nReceived exit signal %d\n", sig);    // Tell the user why the program is exiting
#endif
  pwmWrite    (1, 0);                            // Set the PWM of LED 1 to 0
  digitalWrite(2, 0);                            // Set the state of LED 2 to 0
  pinMode(1, INPUT);                             // Set LED 1 IO as INPUT
  pinMode(2, INPUT);                             // Set LED 2 IO as INPUT
  BrickPiEmergencyStop();                        // Send E Stop to the BrickPi
  serialClose(UART_file_descriptor);             // Close the UART port  
//  signal(SIGINT , BrickPiExitSafely);  Don't bother to re-enable
//  signal(SIGQUIT, BrickPiExitSafely);  Don't bother to re-enable
#ifdef DEBUG
  printf("Exiting.\n", sig);                     // Tell the user that the program is exiting
#endif
  exit(0);                                       // Exit
}

int BrickPiSetup(){
  if(signal(SIGINT , BrickPiExitSafely) == SIG_ERR ||           // Setup exit signal SIGINT
     signal(SIGQUIT, BrickPiExitSafely) == SIG_ERR){            // and SIGQUIT
#ifdef DEBUG
    printf("Exit signal install error\n");                      // If it failed, print error message
#endif
    return -1;                                                  // and return -1
  }
  if(wiringPiSetup() == -1)                                     // If wiringPiSetup failed
    return -1;                                                  //   return -1
  UART_file_descriptor = serialOpen("/dev/ttyAMA0", 500000);    // Open the UART port at 500kbps
  if(UART_file_descriptor == -1)                                // If opening the port failed
    return -1;                                                  //   return -1  
  pinMode(1, PWM_OUTPUT);                                       // LED 1
  pinMode(2, OUTPUT);                                           // LED 2 
  BrickPi.MotorTargetKP[PORT_A] = MOTOR_KP_DEFAULT;             // Set to default
  BrickPi.MotorTargetKP[PORT_B] = MOTOR_KP_DEFAULT;             //      ''
  BrickPi.MotorTargetKP[PORT_C] = MOTOR_KP_DEFAULT;             //      ''
  BrickPi.MotorTargetKP[PORT_D] = MOTOR_KP_DEFAULT;             //      ''
  BrickPi.MotorTargetKD[PORT_A] = MOTOR_KD_DEFAULT;             //      ''
  BrickPi.MotorTargetKD[PORT_B] = MOTOR_KD_DEFAULT;             //      ''
  BrickPi.MotorTargetKD[PORT_C] = MOTOR_KD_DEFAULT;             //      ''
  BrickPi.MotorTargetKD[PORT_D] = MOTOR_KD_DEFAULT;             //      ''
  BrickPi.MotorDead[PORT_A] = MOTOR_DEAD_DEFAULT;               //      ''
  BrickPi.MotorDead[PORT_B] = MOTOR_DEAD_DEFAULT;               //      ''
  BrickPi.MotorDead[PORT_C] = MOTOR_DEAD_DEFAULT;               //      ''
  BrickPi.MotorDead[PORT_D] = MOTOR_DEAD_DEFAULT;               //      ''
  return 0;                                                     // return 0
}

void BrickPiTx(unsigned char dest, unsigned char ByteCount, unsigned char OutArray[]){
  unsigned char tx_buffer[256];
  tx_buffer[0] = dest;
  tx_buffer[1] = dest + ByteCount;
  tx_buffer[2] = ByteCount;  
  unsigned char i = 0;
  while(i < ByteCount){
    tx_buffer[1] += OutArray[i];
    tx_buffer[i + 3] = OutArray[i];
    i++;
  }
  i = 0;
  while(i < (ByteCount + 3)){
    serialPutchar(UART_file_descriptor, tx_buffer[i]);
    i++;
  }
}

int BrickPiRx(unsigned char *InBytes, unsigned char *InArray, long timeout){  // timeout in uS, not mS
  unsigned char rx_buffer[256];
  unsigned char RxBytes = 0;
  unsigned char CheckSum = 0;
  unsigned char i = 0;
  int result;
  unsigned long OrigionalTick = CurrentTickUs();
  while(serialDataAvail(UART_file_descriptor) <= 0){
    if(timeout && ((CurrentTickUs() - OrigionalTick) >= timeout))return -2;
    usleep(100);
  }

  RxBytes = 0;
  while(RxBytes < serialDataAvail(UART_file_descriptor)){                                   // If it's been 1 ms since the last data was received, assume it's the end of the message.
    RxBytes = serialDataAvail(UART_file_descriptor);
    usleep(75);
  }
  
  i = 0;  
  while(i < RxBytes){
    result = serialGetchar(UART_file_descriptor);
    if(result >= 0){
      rx_buffer[i] = result;
    }
    else{      
      return -1;    
    }
    i++;    
  }

  if(RxBytes < 2)
    return -4;
  
  if(RxBytes < (rx_buffer[1] + 2))
    return -6;
  
  CheckSum = rx_buffer[1];
  
  i = 0;
  while(i < (RxBytes - 2)){
    CheckSum += rx_buffer[i + 2];
    InArray[i] = rx_buffer[i + 2];
    i++;
  }
  
  if(CheckSum != rx_buffer[0])
    return -5;
  
  *InBytes = (RxBytes - 2);

  return 0;  
}

#endif