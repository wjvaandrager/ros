/* ---------------------------------------
Hardware control written by Dennis Kaandorp, January 2014

Usage:
To change color:                 RGB red(byte) green(byte) blue(byte);
To turn the loud siren on/off:   sirenLoud 1/0;
To turn the pussy siren on/off:  sirenSoft 1/0;
To open/close the diafragma:     diafragma 1/0;
To turn the strobe on/off:       strobe 1/0;
To change the siren frequency:   sirenFrequency frequency(int);

so for example to change the color to yellow with the strobe on and the diafragma open you would send: "RGB 255 255 0; strobe 1; diafragma 1;"
*/



#include <ros.h>
#include <std_msgs/String.h>

int RGBclockPin = 12;
////Pin connected to DS of 74HC595
int RGBdataPin = 11;
int sirenLoudPin = 13;
int sirenSoftPin = 2;
int Delay = 30;
byte strobeOn = 0;
int strobeDelay = 50; //10 hz
int sirenDelay = 0.2; //2.5 khz soft Siren frequentsy

byte sirenSoftOn = 0;
byte sirenLoudOn = 0;
byte red;
byte green; 
byte blue;

ros::NodeHandle nh;

void messageCb( const std_msgs::String& toggle_msg){
  String message = (const char* )toggle_msg.data;
  String s = splitString(message,' ',0);
  int count = 1;
  while(!s.equals(""){
    if(s.equals("strobe_on")){
      digitalWrite(13, HIGH);   // blink the led
    }else if(s.equals("alarm_on_zacht")){
      sirenSoft();
    }
    else if(s.equals())
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
  nh.subscribe(sub);
}

void loop() {
   nh.spinOnce();
 delay(1);
 
}

void RGB_bulb(){
  if(strobeOn==0){
    shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, red);    //red
    shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, green);  //green
    shiftOut(RGBdataPin, RGBclockPin, MSBFIRST, blue);   //blue
    return;
  }
  else{
    while(strobeOn==1){
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
}

void sirenSoft(){
  while(sirenSoftOn == 1){
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


