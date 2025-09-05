#define triggPin 6
#define echoPin 3
#define buttPin 2

const float SOUND_SPEED = 0.0343;
const float SAFETY_DISTANCE = 100;

volatile unsigned long startTime;
volatile unsigned long endTime;
volatile unsigned long distance;

volatile bool echoReceived = false;
volatile bool robotCrashed = false;

void emitPulse(){
  digitalWrite(triggPin, 0);
  delayMicroseconds(2);
  digitalWrite(triggPin, 1);
  delayMicroseconds(10);
  digitalWrite(triggPin, 0);
}

void echoReceiverHandler(){
  if (digitalRead(echoPin) == 1){ // RISING
    startTime = micros();
  }
  else { // FALLING
    endTime = micros();
    echoReceived = true;
  }
}

void buttonPressedHandler(){
  robotCrashed = true;
  Serial.println("Choqué :(");  
}

void dance(){
  Serial.println("Bailando");
}

void fixRobot(){
  //***
  robotCrashed = false;
}

void setup()
{
  pinMode(triggPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttPin, INPUT_PULLUP);

  pinMode(LED_BUILTIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(echoPin), echoReceiverHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttPin), buttonPressedHandler, FALLING);

  Serial.begin(9600);
}

void buttonListener(){
  if (robotCrashed){
    Serial.println("Arreglando robot");
    delay(1000);
    fixRobot();
  }
}

void usListener(){
  emitPulse();
  if (echoReceived){
    echoReceived = false; 
    distance = (endTime - startTime) * SOUND_SPEED / 2;
    if (distance > SAFETY_DISTANCE){
      digitalWrite(LED_BUILTIN, 0);
      dance();
    }
    else {
      digitalWrite(LED_BUILTIN, 1);
    }  
  }
}

void loop()
{
  buttonListener();
  usListener(); 
}