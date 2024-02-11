#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>


char ssid[] = "Stef hotspot";  //name of network
char pass[] = "mmmmmmmm";      // password of network
char c;
WiFiClient client;
IPAddress server(10, 6, 18, 243);  //network ip adress

//initialising both sensors
const int LEYE = 7;
const int REYE = 8;

#define left_wheel 0A;
#define right_wheel 7A;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, rateError;
double Setpoint;
double input, output;

//PID constants
double kp = -9;
double ki = 0.00007;
double kd = -3;

//initialasing ultrasonic sensor
const int US_TRIG = 15;
const int US_ECHO = 17;

//initialising motors
#define A1 6
#define A2 4
#define A3 3
#define A4 5

#define ENA 10  //enables right
#define ENB 9   //enables left

void setup() {
  Serial.begin(9600);

  //connecting to processing server
  WiFi.begin(ssid, pass);
  client.connect(server, 5200);

  //prints out aurduino ip address
  IPAddress ip = WiFi.localIP();
  Serial.print(ip);

  //setting up pinmode type
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  pinMode(LEYE, INPUT);
  pinMode(REYE, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  distance_calculator();
  setPoint = 20;

  while (!Serial) {
    ;  //waiting for serial port to connect
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (true)
      ;
  }
  Serial.println("IMU initialized!");
}

//function to go backwards
void backwards() {
  analogWrite(ENA, 100);  //motor power reduced to a quater of speed
  analogWrite(ENB, 100);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  Serial.println("Backwards ");
}

//function to go forwards
void forwards() {
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  Serial.println("forwards ");
}

//function to turn left
void left() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 150);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  Serial.println("Left ");
}

//function to turn right
void right() {
  analogWrite(ENA, 150);
  analogWrite(ENB, 100);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  Serial.println("Right ");
}

//function to stop buggy
void stop() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  Serial.println("Stopping ");
}

int distance_calculator() {

  int distance;
  long duration;

  digitalWrite(US_TRIG, LOW);  //turns off incse already on
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);  // turns on
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);  //switched off

  duration = pulseIn(US_ECHO, HIGH);  //measures duration of echo feedback

  distance = duration / 58;

  Serial.print("distance detected: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void track_line() {
  distance_calculator();
  if (digitalRead(REYE) == HIGH && digitalRead(LEYE) == LOW) {  //if right sees black and left sees white
    left();
    distance_calculator();
  }

  else if (digitalRead(LEYE) == HIGH && digitalRead(REYE) == LOW) {  //if left sees black and right sees white
    right();
    distance_calculator();
  }

  else if (digitalRead(REYE) == HIGH && digitalRead(LEYE) == HIGH) {  //if right sees black and left sees black
    forwards();
    distance_calculator();
  }

  else if (digitalRead(REYE) == LOW && digitalRead(LEYE) == LOW) {  // if right sees white and right sees white
    stop();
    distance_calculator();
  }
}

double computePID(double inp) {
  currentTime = millis();                              //get current time
  elapsedTime = (double)(currentTime - previousTime);  //compute time elapsed from previous computation

  error = inp - Setpoint;                         // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError) / elapsedTime;  // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;  //PID output

  lastError = error;           //remember current error
  previousTime = currentTime;  //remember current time

  if (out > 200) {
    out = 200;
  }

  return out;  //have function return the PID output
}

void speed_adjuster() {

  pid_input = distance_calculator();  //take in distance measured by Ultrasonic

  pid_output = computePID(pid_input);

  analogWrite(9, pid_output);
  analogWrite(10, pid_output);

  delayMicroseconds(1);
}

void IMU_stuff() {
}

void loop() {
  char cmd = client.read();  //read info from processing (aka server)
  if (isAlpha(cmd)) {        //making sure info is a character
    c = cmd;
  }
  if (c == 'g') {                       //go button from processing
    if (distance_calculator() <= 20) {  //if obstacle less than 20cm away
      stop();
      client.write("Obstacle in the way!!");
    } else {
      client.write("Going");  //send message to proccessing (aka server)
      track_line();           //buggy will follow the line otherwise
      distance_calculator();  //get distance
      speed_adjuster();
    }
  }
  if (c == 's') {                //stop button from processing
    client.write("Stopping!!");  //sends message to proccessing (aka server)
    stop();
  }
}
