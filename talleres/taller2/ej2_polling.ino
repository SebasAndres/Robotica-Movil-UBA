#define triggPin 6
#define echoPin 2

const float SOUND_SPEED = 0.0343;

volatile unsigned long  startTime;
volatile unsigned long  endTime;
volatile bool echoReceived = false;
volatile float distance; // (cm)

void emitPulse(){
  digitalWrite(triggPin, 0);
  delayMicroseconds(2);
  digitalWrite(triggPin, 1);
  delayMicroseconds(10);
  digitalWrite(triggPin, 0);
}

void setup()
{
  pinMode(triggPin, OUTPUT);
  pinMode(echoPin, INPUT);  
  Serial.begin(9600);
}

void loop()
{	  
  emitPulse();  

  while(digitalRead(echoPin)==0){}
  startTime = micros();
  while(digitalRead(echoPin)==1){}
  endTime = micros();
  
  distance = (endTime - startTime) * SOUND_SPEED / 2;
  Serial.println(distance);  
}
