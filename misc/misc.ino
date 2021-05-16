#include <avdweb_AnalogReadFast.h>
#include <DIO2.h> // install the library DIO2

#define VOLUME_PIN A0

int volume_analog = 0;
byte precision = 1;
int num = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //LEDS OUT
  for(int i = 22; i < 38; i += 2){
     pinMode2(i, OUTPUT);
     digitalWrite2(i, HIGH);
  }

  //LEDS IN
  for(int i = 40; i <= 52; i += 2){
     pinMode2(i, OUTPUT);
     digitalWrite2(i, HIGH);
  }

  for(int i = 2; i <= 4; i++){
     pinMode2(i, OUTPUT);
     digitalWrite2(i, HIGH);
  }

  //BUTTONS OUT
  for(int i = 39; i <= 53; i += 2){
     pinMode2(i, OUTPUT);
     digitalWrite2(i, LOW);
  }

  //BUTTONS IN
  for(int i = 23; i <= 37; i += 2){
     pinMode2(i, INPUT_PULLUP);
  }
}

void loop() {

  if(num < 10){
    num++;
  } else {
    num = 0;
  }

  //LEDS IN
  for(int i = 40, j = 0; i <= 52; i += 2, j++){
    if(j == num){
      digitalWrite2(i, LOW);
    }else{
      digitalWrite2(i, HIGH);
    }
  }

  for(int i = 2; i <= 4; i++, j++){
     if(j == num){
      digitalWrite2(i, LOW);
    }else{
      digitalWrite2(i, HIGH);
    }
  }
  
  /*switch(num){
    case 0:
      digitalWrite2(8, HIGH);
      digitalWrite2(9, LOW);
      digitalWrite2(10, LOW);
      digitalWrite2(11, LOW);
      break;
    case 1:
      digitalWrite2(8, LOW);
      digitalWrite2(9, HIGH);
      digitalWrite2(10, LOW);
      digitalWrite2(11, LOW);
      break;
    case 2:
      digitalWrite2(8, LOW);
      digitalWrite2(9, LOW);
      digitalWrite2(10, HIGH);
      digitalWrite2(11, LOW);
      break;
    case 3:
      digitalWrite2(8, LOW);
      digitalWrite2(9, LOW);
      digitalWrite2(10, LOW);
      digitalWrite2(11, HIGH);
      break;
  }
  
  delay(250);*/
  int buttons[8];
  
  for(int i = 23, j = 0; i <= 37; i += 2, j++){
     buttons[j] = digitalRead2(i);
  }
  
  volume_analog = analogReadFast(VOLUME_PIN);
  volume_analog /= 10;
  float volume_final = ((float)volume_analog/71.0)*100.0;
  
  char floatBuffer[10];
  dtostrf(volume_final, precision+3, precision, floatBuffer);
  
  char out[32];
  sprintf(out, "Volume: %s", floatBuffer);
  //Serial.println(out);
  
  for(int i = 0; i < 8; i++){
    sprintf(out, "Button[%d]: %d", i, buttons[i]);
    Serial.println(out);
  }

  delay(500);
}
