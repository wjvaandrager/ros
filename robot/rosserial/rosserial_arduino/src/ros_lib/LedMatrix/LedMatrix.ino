//**************************************************************//
//  Name    : RGB color for RGB dot matrix                              
//  Author  : D.C. Kaandorp
//  Date    : 14-01-2014                               
//  Version : 2.0                                                                     
//****************************************************************

#include <ros.h>
#include <std_msgs/String.h>

#include <string.h>

ros::NodeHandle  nh;
//Pin connected to ST_CP of 74HC595
int latchPin = 12;
//Pin connected to SH_CP of 74HC595
int clockPin = 11;
////Pin connected to DS of 74HC595
int dataPin = 10;
int delayTime = 2;
int frameCycles = 20;
byte mad2[80] = {24, 8, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 254, 0, 0, 195, 0, 0, 127, 0, 0, 254, 0, 0, 129, 0, 0, 127, 0, 0, 254, 0, 0, 0, 0, 0, 127, 0, 0, 126, 0, 0, 0, 0, 0, 126, 0, 0, 62, 0, 0, 0, 0, 0, 124, 0, 0, 14, 0, 0, 0, 0, 0, 112, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte smiley1[80] = {24, 8, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 129, 129, 129, 128, 128, 128, 131, 131, 131, 129, 129, 129, 193, 193, 193, 255, 255, 255, 129, 129, 129, 255, 255, 255, 255, 255, 255, 129, 129, 129, 255, 255, 255, 254, 254, 254, 0, 0, 0, 127, 127, 127, 124, 124, 124, 0, 0, 0, 62, 62, 62, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte standaard[80] = {24, 8, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 129, 129, 129, 255, 255, 255, 255, 255, 255, 129, 129, 129, 255, 255, 255, 255, 255, 255, 129, 129, 129, 255, 255, 255, 255, 255, 255, 129, 129, 129, 255, 255, 255, 255, 255, 255, 129, 129, 129, 255, 255, 255, 254, 254, 254, 0, 0, 0, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        
int check = -1;
int number = 0;
int nColors = 1;
int nFrames = 1;
int height = 8;
int width = 24;

void messageCb( const std_msgs::String& toggle_msg){

  String message = (const char* )toggle_msg.data;
 
  
  int array = message.toInt();
  if (array == 1){
    check = 1;
  }else if (array == 2){
    check = 2;
  }else if (array == 3){
    check = 3;
  }else{
    check = 0;
  }
  
}

ros::Subscriber<std_msgs::String> sub("/arduino/led", messageCb );


void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);  
  digitalWrite(13,HIGH);
  digitalWrite(13,LOW);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  for(int i=0; i<7; i++){
    digitalWrite(latchPin,LOW);
    digitalWrite(latchPin,HIGH);
  }
  nh.initNode();
  nh.subscribe(sub);

    //open("/home/dennis/workspace/imageToHardware.txt");

}


void loop() {
 nh.spinOnce();

 if(check == 0){
  clear();
  check = -1;
 }
 
 if(check == 1){
  for(int j = 0; j<nFrames;j++){
    for(int k = 0; k<frameCycles; k++){  
   
      for(int i = 0; i<height*nColors*3*3; i=i+9){
      
        PORTB &= ~_BV(4);//digitalWrite(latchPin, LOW);
        shiftOutFast(mad2[i+4+j*height*nColors*3]);
        shiftOutFast(mad2[i+1+4+j*height*nColors*3]);
        shiftOutFast(mad2[i+2+4+j*height*nColors*3]);
              shiftOutFast(mad2[i+3+4+j*height*nColors*3]);
        shiftOutFast(mad2[i+4+4+j*height*nColors*3]);
              shiftOutFast(mad2[i+5+4+j*height*nColors*3]);
        shiftOutFast(mad2[i+6+4+j*height*nColors*3]);
              shiftOutFast(mad2[i+7+4+j*height*nColors*3]);
        shiftOutFast(mad2[i+8+4+j*height*nColors*3]);
   
        PORTB |= _BV(4);//digitalWrite(latchPin, HIGH);
        delay(delayTime);
      }
    }
  }
 }

 if(check == 2){
  for(int j = 0; j<nFrames;j++){
    for(int k = 0; k<frameCycles; k++){  
   
      for(int i = 0; i<height*nColors*3*3; i=i+9){
      
        PORTB &= ~_BV(4);//digitalWrite(latchPin, LOW);
        shiftOutFast(smiley1[i+4+j*height*nColors*3]);
        shiftOutFast(smiley1[i+1+4+j*height*nColors*3]);
        shiftOutFast(smiley1[i+2+4+j*height*nColors*3]);
              shiftOutFast(smiley1[i+3+4+j*height*nColors*3]);
        shiftOutFast(smiley1[i+4+4+j*height*nColors*3]);
              shiftOutFast(smiley1[i+5+4+j*height*nColors*3]);
        shiftOutFast(smiley1[i+6+4+j*height*nColors*3]);
              shiftOutFast(smiley1[i+7+4+j*height*nColors*3]);
        shiftOutFast(smiley1[i+8+4+j*height*nColors*3]);
   
        PORTB |= _BV(4);//digitalWrite(latchPin, HIGH);
        delay(delayTime);
      }
    }
  }
 }
  
 if(check == 3){
  for(int j = 0; j<nFrames;j++){
    for(int k = 0; k<frameCycles; k++){  
   
      for(int i = 0; i<height*nColors*3*3; i=i+9){
      
        PORTB &= ~_BV(4);//digitalWrite(latchPin, LOW);
        shiftOutFast(standaard[i+4+j*height*nColors*3]);
        shiftOutFast(standaard[i+1+4+j*height*nColors*3]);
        shiftOutFast(standaard[i+2+4+j*height*nColors*3]);
              shiftOutFast(standaard[i+3+4+j*height*nColors*3]);
        shiftOutFast(standaard[i+4+4+j*height*nColors*3]);
              shiftOutFast(standaard[i+5+4+j*height*nColors*3]);
        shiftOutFast(standaard[i+6+4+j*height*nColors*3]);
              shiftOutFast(standaard[i+7+4+j*height*nColors*3]);
        shiftOutFast(standaard[i+8+4+j*height*nColors*3]);
   
        PORTB |= _BV(4);//digitalWrite(latchPin, HIGH);
        delay(delayTime);
      }
    }
  }
 }
 delay(1);
}

void clear(){
  PORTB &= ~_BV(4);//digitalWrite(latchPin, LOW);
  for(int i = 0; i < 9; i++){
    shiftOutFast(0);
  }
  PORTB |= _BV(4);//digitalWrite(latchPin, HIGH);
  for(int i=0; i<7; i++){
    digitalWrite(latchPin,LOW);
    digitalWrite(latchPin,HIGH);
  }
}

void shiftOutFast(byte data){
  for (byte mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
    if (data & mask){ // if bitwise AND resolves to true
      PORTB |= _BV(2);//digitalWrite(dataPin, HIGH);
      PORTB |= _BV(3);//digitalWrite(clockPin, HIGH);
      PORTB &= ~_BV(3);//digitalWrite(latchPin, LOW);
    }
    else{ //if bitwise and resolves to false
      PORTB &= ~_BV(2);//digitalWrite(dataPin, LOW);
      PORTB |= _BV(3);//digitalWrite(clockPin, HIGH);
      PORTB &= ~_BV(3);//digitalWrite(latchPin, LOW);
    }
  }
}







