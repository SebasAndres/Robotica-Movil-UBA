#define BUMPER_UP_PIN 2
#define BUMPER_DOWN_PIN 3

void handleColision(){
  Serial.println("* Frenando motores!!");
}


void setup()
{
  pinMode(BUMPER_UP_PIN, INPUT_PULLUP);
  pinMode(BUMPER_DOWN_PIN, INPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(BUMPER_UP_PIN), handleColision, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUMPER_DOWN_PIN), handleColision, RISING);

  Serial.begin(9600);
}

void loop()
{  
  //Serial.println(digitalRead(BUMPER_DOWN_PIN));
}

/*
¿Hay otra configuración de la interrupción para la cual el algoritmo siga
funcionando? ¿Cuál/Cuáles?
|--> Si, usando PULL_DOWN y RISING. O directamente usando CHANGE.
*/