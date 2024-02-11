#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>  //needed for the Gold Challenge

char ssid[] = "Aoife_iphone";  //name of network
char pass[] = "NissaNpwx";     // password of network
 
char c;

WiFiClient client;
IPAddress server(172, 20, 10, 3);  //network ip address

//initialising both sensors
const int LEYE = 7;
const int REYE = 8;

//initialasing ultrasonic sensor
const int US_TRIG = 15;
const int US_ECHO = 17;

// Initialising encoder
const int enc_pin = 11;

// Initialising IMU
float x, y, z;
float x_cur = 0;
float x_prev = 0;
float y_cur = 0;
float y_prev = 0;
float z_cur = 0;
float z_prev = 0;
unsigned long count_cur = 0;
unsigned long count_prev = 0;

//initialising motors
#define A1 6
#define A2 4
#define A3 3
#define A4 5

#define ENA 10  //enables right
#define ENB 9   //enables left

//PID values
int kp = -16;
int ki = -0.000007;
int kd = -2;

// Times for PID
unsigned long currentTime;
unsigned long previousTime;

// Times for Encoder
unsigned long curr_time = 0;
unsigned long prev_time = 0;

// Times for distance calculator to get changing_speed
unsigned long c_time = 0;
unsigned long p_time = 0;

// PID and speed control variables
double elapsedTime;
double error;
double lastError;
double cumError;
double rateError;
double Setpoint;
double pid_input;
double pid_output;
int speed_output;

// Encoder Variables
double rps;
double buggy_speed;
double changing_speed;  // Speed of obstacle depending on distance measurement change by Ultrasonic
double obst_speed;      // Actual speed of obstacle
unsigned long count;
int prev_dist;  // For a crude estimate of the objects speed relative to the buggy
double b_speed;


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

  pinMode(enc_pin, INPUT_PULLUP);

  distance_calculator();

  while (!Serial)
    ;
  //making sure IMU does connect
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1)
      ;
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
}

//function to go backwards
void backwards() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
}

//function to go forwards
void forwards() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
}

//function to turn left
void left() {
  analogWrite(ENA, speed_output);
  analogWrite(ENB, (speed_output - 100));
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
}

//function to turn right
void right() {
  analogWrite(ENA, (speed_output - 100));
  analogWrite(ENB, speed_output);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
}

//function to stop buggy
void stop() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
}

int distance_calculator() {

  int distance;
  long duration;

  digitalWrite(US_TRIG, LOW);  //turns off incase already on
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);  // turns on
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);  //switched off

  duration = pulseIn(US_ECHO, HIGH);  //measures duration of echo feedback

  distance = duration / 58;

  c_time = millis();
  if (c_time - p_time >= 500) {
    changing_speed = (distance - prev_dist) / 0.5;

    p_time = c_time;
    prev_dist = distance;
  }
  return distance;
}

void track_line() {

  distance_calculator();

  if (digitalRead(REYE) == HIGH && digitalRead(LEYE) == LOW) {  //if right sees black and left sees white
    left();
  }

  else if (digitalRead(LEYE) == HIGH && digitalRead(REYE) == LOW) {  //if left sees black and right sees white
    right();
  }

  else if (digitalRead(REYE) == HIGH && digitalRead(LEYE) == HIGH) {  //if right sees black and left sees black
    forwards();

    speedAdjuster();

  }

  else if (digitalRead(REYE) == LOW && digitalRead(LEYE) == LOW) {  // if right sees white and right sees white
    stop();
  }
}

double computePID(double inp) {
  Setpoint = 17;

  currentTime = micros();                              //get current time
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

void speedAdjuster() {

  pid_input = distance_calculator();  //take in distance measured by Ultrasonic

  pid_output = computePID(pid_input);

  // setting new speeds for left and right motors
  analogWrite(9, pid_output);
  analogWrite(10, pid_output);

  delayMicroseconds(1);
}

double objectTracker() {

  curr_time = millis();

  //count = 0;
  if (curr_time - prev_time >= 1000) {
    detachInterrupt(digitalPinToInterrupt(enc_pin));

    rps = 1000.0 * ((count / 8.0) / (curr_time - prev_time)) / 120.0;

    prev_time = curr_time;
    count = 0;

    attachInterrupt(digitalPinToInterrupt(enc_pin), countIncrement, RISING);

    buggy_speed = (6.7 * PI * rps);
    b_speed = buggy_speed;

    obst_speed = b_speed + changing_speed;
    if (obst_speed < 0) {
      obst_speed = 0;
    }
  }
  return buggy_speed;
}

double object_speed() {
  objectTracker();
  return obst_speed;
}

void countIncrement() {
  count++;
}

void buggyInAir() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
}

bool buggyHeldBack() {
  x_cur = x;
  y_cur = y;
  z_cur = z;
  count_cur = count;
 
 // if buggy is held very still but wheels are still moving
  if ((x_cur == x_prev) && (y_cur == y_prev) && (count_cur != count_prev) && (count_cur > 0)) {
    buggy_speed = 0;
    return true;
  }

  x_prev = x_cur;
  y_prev = y_cur;
  z_prev = z_cur;
  count_prev = count_cur;
}

void loop() {

  buggyInAir();

  char cmd = client.read();  //read info from processing (aka server)
  if (isAlpha(cmd)) {        //making sure info is a character
    c = cmd;
  }

  if (c == 'g') {  //go button from processing

    if ((x > 0.6) || (x < -0.6) || (y > 0.6) || (y < -0.6)) {  //testing if buggy is picked up at a large angle
      Serial.println("I'm getting dizzy...   :(\n");
      client.print("I'm getting dizzy...   :(\n");
    }

    else if ((x > 0.1) || (x < -0.1) || (y > 0.1) || (y < -0.1)) {  //testing if buggy is just picked up
      Serial.println("Put me down!!!   >:(\n");
      client.print("Put me down!!!   >:(\n");
    }

    else if (distance_calculator() <= 10) {  //if obstacle less than 10cm away
      String obst_in_way = "Obstacle in the way!!";
      stop();
      client.print(obst_in_way + "\n");
    }

    else {
      track_line();           //buggy will follow the line otherwise
      distance_calculator();  //get distance

      double bug_speed = objectTracker();
      double obstacle_speed = object_speed();
      int obstacle_distance = distance_calculator();

      if (obstacle_distance >= 40) {
        String no_obst = "No obstacle detected in range!!";
        client.print(no_obst + "\n");
        obstacle_speed = 0;
      }

      else if (buggyHeldBack() == true) {
        client.print("Let me go!   >:|\n");
      }

      else {
        client.print("Distance Detected: " + String(obstacle_distance) + " cm" + "\n");
      }

      client.print("Buggy Speed " + String(bug_speed) + " cm/s" + "\n");
      client.print("Obstacle Speed " + String(obstacle_speed) + " cm/s" + "\n");
    }
  }
  if (c == 's') {  //stop button from processing
    stop();
  }
}
