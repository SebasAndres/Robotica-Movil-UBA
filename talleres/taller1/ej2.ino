#define BUZZER 12

#define ROW1 8
#define ROW2 7
#define ROW3 6
#define ROW4 5

#define COL1 4
#define COL2 3
#define COL3 2
#define COL4 1

void setup()
{
  
  pinMode(ROW1, INPUT_PULLUP);
  pinMode(ROW2, INPUT_PULLUP);
  pinMode(ROW3, INPUT_PULLUP);
  pinMode(ROW4, INPUT_PULLUP);
  pinMode(COL1, OUTPUT);
  pinMode(COL2, OUTPUT);
  pinMode(COL3, OUTPUT);
  pinMode(COL4, OUTPUT);

  digitalWrite(COL1, 1);
  digitalWrite(COL2, 1);
  digitalWrite(COL3, 1);
  digitalWrite(COL4, 1);
  
  Serial.begin(9600);
}

int getNumber(){
  int num = -1;

  digitalWrite(COL1, 0);
  if (!digitalRead(ROW1)){
    num = 1;
  }
  else if (!digitalRead(ROW2)){
    num = 4;
  }  
  else if (!digitalRead(ROW3)){
    num = 7;
  }
  digitalWrite(COL1, 1);

  digitalWrite(COL2, 0);
  if (!digitalRead(ROW1)){
    num = 2;
  }
  else if (!digitalRead(ROW2)){
    num = 5;
  }  
  else if (!digitalRead(ROW3)){
    num = 8;
  }
  digitalWrite(COL2, 1);

  digitalWrite(COL3, 0);
  if (!digitalRead(ROW1)){
    num = 3;
  }
  else if (!digitalRead(ROW2)){
    num = 6;
  }  
  else if (!digitalRead(ROW3)){
    num = 9;
  }
  digitalWrite(COL3, 1);

  digitalWrite(COL1, 1);
  digitalWrite(COL2, 1);
  digitalWrite(COL4, 1);
  
  return num;
}


void loop()
{
  int number = getNumber();  
  Serial.println(number);
  tone(BUZZER, number+50);
}
