// LED Display for TEST & Debug

#define T_LED01 A6
#define T_LED02 A7
#define T_LED03 A14
#define T_LED04 A15

// the setup function runs once when you press reset or power the board
void LED_Init() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(T_LED01, OUTPUT);
  pinMode(T_LED02, OUTPUT);
  pinMode(T_LED03, OUTPUT);
  pinMode(T_LED04, OUTPUT);
}

void LED01(int level){
  if( level == HIGH)
      digitalWrite(T_LED01, HIGH); 
  else
    digitalWrite(T_LED01, LOW); 
  return;
}

void LED02(int level){
  if( level == HIGH)
      digitalWrite(T_LED02, HIGH); 
  else
    digitalWrite(T_LED02, LOW); 
  return;
}

void LED03(int level){
  if( level == HIGH)
      digitalWrite(T_LED03, HIGH); 
  else
    digitalWrite(T_LED03, LOW); 
  return;
}

void LED04(int level){
  if( level == HIGH)
      digitalWrite(T_LED04, HIGH); 
  else
    digitalWrite(T_LED04, LOW); 
  return;
}
