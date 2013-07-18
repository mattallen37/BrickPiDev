/*
*  Matthew Richardson
*  matthewrichardson37<at>gmail.com
*  http://mattallen37.wordpress.com/
*  Initial date: June 4, 2013
*  Last updated: July 18, 2013
*
*  You may use this code as you wish, provided you give credit where it's due.
*
*  This is a program for testing the RPi BrickPi drivers.
*/

#include <stdio.h>
#include <math.h>
#include <time.h>

#include "tick.h"

#include <wiringPi.h>

#include "BrickPi.h"

//#include <unistd.h>  
//#include <errno.h>  
//#include <stdio.h>  
//#include <stdlib.h>  
#include <linux/i2c-dev.h>  
//#include <sys/ioctl.h>  
#include <fcntl.h>

// gcc -o program "/home/pi/dbrpi/C/Test BrickPi LEDs.c" -lrt -lm -L/usr/local/lib -lwiringPi
// ./program

int result;

int main() {
  ClearTick();

  result = BrickPiSetup();
  printf("BrickPiSetup: %d\n", result);
  if(result)
    return 0;
  
  int l;
  int i;
  while(1){
    for(l = 0; l < 5; l++){
      for(i = 0; i < 5; i++){
        BrickPiSetLed(LED_1, LED_ON);
        delay(50);
        BrickPiSetLed(LED_1, LED_OFF);
        delay(50);
      }
      for(i = 0; i < 5; i++){
        BrickPiSetLed(LED_2, LED_ON);
        delay(50);
        BrickPiSetLed(LED_2, LED_OFF);
        delay(50);
      }
    }
    
    for(l = 0; l < 10; l++){
      BrickPi.LED[LED_1] = LED_OFF;
      BrickPi.LED[LED_2] = LED_ON;
      BrickPiUpdateLEDs();
      delay(250);
      BrickPi.LED[LED_1] = LED_ON;
      BrickPi.LED[LED_2] = LED_OFF;
      BrickPiUpdateLEDs();
      delay(250);
    }
    
    for(l = 0; l < 2; l++){
      BrickPi.LED[LED_2] = LED_ON;    
      for (i = 0; i < 1024; i++)
      {
        BrickPi.LED[LED_1] = i;
        BrickPiUpdateLEDs();
        delay(1);
      }
      BrickPi.LED[LED_2] = LED_OFF;
      for (i = 1023; i >= 0; i--)
      {
        BrickPi.LED[LED_1] = i;
        BrickPiUpdateLEDs();
        delay(1);
      }
    }
  }
  return 0;
}