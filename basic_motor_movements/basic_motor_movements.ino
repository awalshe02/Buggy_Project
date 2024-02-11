const int LEYE = 9;
const int REYE = 10;
const int carspeed = 120;

#define A1 6
#define A2 4
#define A3 3
#define A4 5
#define EN1 14  //enables right
#define EN3 21  // left

void backwards() { 
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  Serial.println("Backwards ");
}

void forwards() {
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  Serial.println("forwards ");
}

void left() {
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  Serial.println("Left ");
}

void right() {
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  Serial.println("Right ");
}

void stop() {
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  Serial.println("Stop ");
}


void setup() {
  Serial.begin(9600);
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  rotateMotor(0, 0);
  // changing to higher frequency for pmw so motor runs at a more controlled rate
}

void loop() {
  int rightSensor = digitalRead(REYE);
  int leftSensor = digitalRead(LEYE);
  //no black line detected, go staright
  if (rightSensor == LOW && leftSensor == LOW) {
    rotateMotor(carspeed, carspeed);
    //forward();
  }
  //right sensor detects line so turn right
  else if (rightSensor == HIGH && leftSensor == LOW) {
    rotateMotor(-carspeed, carspeed);
    //Right();
  }
  //left sensor detects so turns left
  else if (rightSensor == LOW && leftSensor == HIGH) {
    rotateMotor(carspeed, -carspeed);
    //Left();
  } else {
    rotateMotor(0, 0);
    //stop();
  }
}

void rotateMotor(int rightspeed, int leftspeed) {
  if (rightspeed < 0) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
  } else if (rightspeed > 0) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
  } else {
    digitalWrite(A1, LOW);
    digitalWrite(A2, LOW);
  }

  if (leftspeed < 0) {
    digitalWrite(A3, LOW);
    digitalWrite(A4, HIGH);
  } else if (leftspeed > 0) {
    digitalWrite(A3, HIGH);
    digitalWrite(A4, LOW);
  } else {
    digitalWrite(A3, LOW);
    digitalWrite(A4, LOW);
  }
  analogWrite(EN1, abs(rightspeed));
  analogWrite(EN3, abs(leftspeed));
}
