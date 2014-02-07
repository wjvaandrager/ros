double optel = 0;
int middelSize = 2;
double middel[2];
double middel2[2];
double gemiddelde;
double gemiddelde2;
double distance;
double distance2;
int alreadyZero = 0;
int alreadyZero2 = 0;
double sensorwaarde1;
#define trigPin 7
#define echoPin 6
#define trigPin2 5
#define echoPin2 4


void setup() {
  // initialize serial communication at 57600 bits per second:
  Serial.begin(57600);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(trigPin2,OUTPUT);  
  pinMode(echoPin2,INPUT);
}

void zeroArray(){
  if(alreadyZero == 0){
    for(int i = 0; i < middelSize; i++){
      middel[i] = 0;
    }
    alreadyZero = 1;
  }
}

void zeroArray2(){
  if(alreadyZero2 == 0){
    for(int i = 0; i < middelSize; i++){
      middel2[i] = 0;
    }
    alreadyZero2 = 1;
  }
}


void loop() {
 

  zeroArray();   
  double duration;
  duration = gemiddelde;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  for(int i = middelSize - 1; i >= 0; i--){
    
    if (i==0){
    middel[i] = distance;
    optel += middel[i];
    }
    else{
    middel[i] = middel[i-1];
    optel += middel[i];
    }
  }
  
 
  
  gemiddelde = double(optel)/double(middelSize);
  optel = 0;
  delay(20);

  double duration2;
  zeroArray2();   
  duration2 = gemiddelde2;
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2); 
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2) / 29.1;
  for(int i = middelSize - 1; i >= 0; i--){
    
    if (i==0){
    middel2[i] = distance2;
    optel += middel2[i];
    }
    else{
    middel2[i] = middel2[i-1];
    optel += middel2[i];
    }
  }
  double a1 = 0;
  double a2 = 0;
  if (gemiddelde>= 300 || gemiddelde <= 3){
    a1 = -1; 
  }
  else {
    a1 = gemiddelde;
  }
  if (gemiddelde2>= 300 || gemiddelde2 <= 3){
    a2 = -1;
  }
  else {
    a2 = gemiddelde2;
  }
  // where result = 0.12345

  char format_result_end[] = "]#";
  char format_result_start[] = "#S|TEMPFIX|[";
  char buffer_result[sizeof(gemiddelde)+sizeof(format_result_start)+sizeof(format_result_end)];

  sprintf(buffer_result, "%s%.3lf]#", format_result_start, gemiddelde, format_result_end);
  // the 3 in .3lf can be ajusted as the numbers you want after zero.

  Serial.println(buffer_result);
}
  gemiddelde2 = double(optel)/double(middelSize);
  optel = 0;
  delay(20);
}
  
  
  
  
  
  
  
  
 

