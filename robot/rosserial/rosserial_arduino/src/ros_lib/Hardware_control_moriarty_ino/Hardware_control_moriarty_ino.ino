/* ---------------------------------------
Hardware control written by Dennis Kaandorp, January 2014

Usage:
To change color:                 RGB red(byte) green(byte) blue(byte);
To turn the loud siren on/off:   sirenLoud_on;
To turn the pussy siren on/off:  sirenSoft_off;
To open/close the diafragma:     diafragma 1/0;
To turn the strobe on/off:       strobe 1/0;
To change the siren frequency:   sirenFrequency frequency(int);

so for example to change the color to yellow with the strobe on and the diafragma open you would send: "RGB 255 255 0; strobe 1; diafragma 1;"
*/



#include <ros.h>
#include <std_msgs/String.h>

int RGBclockPin = 11;
////Pin connected to DS of 74HC595
int RGBdataPin = 12;
int sirenLoudPin = 13;
int sirenSoftPin = 10;
int Delay = 30;
byte strobeOn = 0;
int strobeDelay = 50; //10 hz
int sirenDelay = 0.1; //2.5 khz soft Siren frequentsy

byte sirenSoftOn = 0;
byte sirenLoudOn = 0;
byte red= 255;
byte green=255; 
byte blue=255;
ros::NodeHandle nh;
std_msgs::String test;
ros::Publisher p("/test",&test);


void messageCb( const std_msgs::String& toggle_msg){
  String message = (const char* )toggle_msg.data;
  String s = splitString(message,' ',0);
  int count = 1;
  while(!s.equals("")){
    if(s.equals("strobe_on")){
      strobeOn= 1;
      RGB_bulb();
    }else if(s.equals("strobe_off")){
      strobeOn = 0;
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0);    //red
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0);  //green
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0);   //blue
    }else if(s.equals("sirenSoft_on")){
      sirenSoftOn = 1;
      sirenSoft();
    }else if(s.equals("sirenSoft_off")){
      sirenSoftOn = 0;
    }else if(s.equals("sirenLoud_on")){
      sirenLoudOn = 1;
    }else if(s.equals("sirenLoud_off")){
      sirenLoudOn = 0;
    }
    s = splitString(message,' ',count);
    count++;
  }
}

ros::Subscriber<std_msgs::String> sub("/arduino/comb", messageCb );


void setup(){ 
  pinMode(13, OUTPUT);
  Serial.begin(9600);  
  pinMode(RGBclockPin, OUTPUT);
  pinMode(RGBdataPin, OUTPUT);
  pinMode(sirenLoudPin, OUTPUT);
  pinMode(sirenSoftPin, OUTPUT); 
  digitalWrite(sirenSoftPin, LOW);
  digitalWrite(sirenLoudPin, LOW);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);
  
}

void loop() {
 nh.spinOnce();
 sirenSoft();
 RGB_bulb();
}

void RGB_bulb(){
  if(strobeOn==1){
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, red); //red
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST,green);  //green
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, blue); //blue
      delay(strobeDelay);
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0); //red
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0);  //green
      shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, 0); //blue
      delay(strobeDelay);
  }
}

void sirenSoft(){
  if(sirenSoftOn == 1){
    digitalWrite(sirenSoftPin, HIGH);
    delay(sirenDelay);
    digitalWrite(sirenSoftPin, LOW);
    delay(sirenDelay);
  }
}

String splitString(String s, char parser,int index){
  String rs='\0';
  int parserIndex = index;
  int parserCnt=0;
  int rFromIndex=0, rToIndex=-1;

  while(index>=parserCnt){
    rFromIndex = rToIndex+1;
    rToIndex = s.indexOf(parser,rFromIndex);

    if(index == parserCnt){
      if(rToIndex == 0 || rToIndex == -1){
        return '\0';
      }
      return s.substring(rFromIndex,rToIndex);
    }
    else{
      parserCnt++;
    }

  }
  return rs;
}



