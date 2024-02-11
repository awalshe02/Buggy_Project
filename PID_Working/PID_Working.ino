#include <Arduino_LSM6DS3.h>
#include <WiFiNINA.h>

char ssid[] = "Stef hotspot";  //name of network
char pass[] = "mmmmmmmm";      // password of network
char c;
WiFiClient client;
IPAddress server(192, 168, 94, 111);  //network ip adress

//initialising both sensors
const int LEYE = 7;
const int REYE = 8;

//initialasing ultrasonic sensor
const int US_TRIG = 15;
const int US_ECHO = 17;


// Initialising encoder
const int enc_pin = 11;

//initialising motors
#define A1 6
#define A2 4
#define A3 3
#define A4 5

#define ENA 10  //enables right
#define ENB 9   //enables left


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
unsigned long secs_span;

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
double test = 0;

// IMU Variables
float current_acceleration;
float current_gyroscope;
float ax, ay, az;
float gx, gy, gz;


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

  while (!Serial) {
    ;
  }
  if (!IMU.begin()) {
    Serial.println("failed to initialize IMU!");
    while (true)
      ;
  }
  Serial.println("IMU initialized!");

  IMU_jazz();
  current_acceleration = IMU.readAcceleration(ax, ay, az);
  current_gyroscope = IMU.readGyroscope(gx, gy, gz);
}

//function to go backwards
void backwards() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  //Serial.println("Backwards ");
}

//function to go forwards
void forwards() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  //Serial.println("forwards ");
}

//function to turn left
void left() {
  analogWrite(ENA, speed_output);
  analogWrite(ENB, (speed_output - 100));
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, LOW);
  digitalWrite(A4, HIGH);
  //Serial.println("Left ");
}

//function to turn right
void right() {
  analogWrite(ENA, (speed_output - 100));
  analogWrite(ENB, speed_output);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
  //Serial.println("Right ");
}

//function to stop buggy
void stop() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  // Serial.println("Stopping ");
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
    secs_span = (c_time - p_time) / 1000;
    changing_speed = (distance - prev_dist) / 0.5;

    p_time = c_time;
    prev_dist = distance;
  }

  //Serial.print(" distance detected: ");
  //Serial.print(distance);
  //Serial.println(" cm");
  return distance;
}

void IMU_jazz() {
  float ax, ay, az;
  float gx, gy, gz;
 

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
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
  //Serial.print(out);

  lastError = error;           //remember current error
  previousTime = currentTime;  //remember current time

  if (out > 200) {
    out = 200;
  }

  return out;  //have function return the PID output
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

void loop() {
  char cmd = client.read();  //read info from processing (aka server)
  if (isAlpha(cmd)) {        //making sure info is a character
    c = cmd;
  }
  if (c == 'g') {                       //go button from processing
    if (distance_calculator() <= 10) {  //if obstacle less than 20cm away
      String obst_in_way = "Obstacle in the way!!";
      stop();
      client.print(obst_in_way + "\n");
    }

    else {
      //client.write("Going"); //send message to proccessing (aka server)
      track_line();           //buggy will follow the line otherwise
      distance_calculator();  //get distance
      //speedAdjuster();        //adjust speed
      double bug_speed = objectTracker();
      double obstacle_speed = object_speed();
      int obstacle_distance = distance_calculator();

      //Serial.println("Current Time: " + String(curr_time) + " milliseconds");
      //Serial.println("Previous Time: " + String(prev_time) + " milliseconds");
      //Serial.println("Time difference: " + String((curr_time - prev_time) / 1000.0) + " seconds");
      //Serial.println("Count: " + String(count));
      //Serial.println("Testing: " + String(test));
      //Serial.println("Revs per sec: " + String(rps));

      IMU_jazz();
      float new_acc = IMU.accelerationAvailable(), new_gyr = IMU.gyroscopeAvailable();
      if (current_acceleration != new_acc && current_gyroscope != new_gyr){
        client.print("Put me down >:( ");
      }      
      IMU.gyroscopeSampleRate()
      

      if (obstacle_distance >= 40) {
        String no_obst = "No obstacle detected in range!!";
        client.print(no_obst + "\n");
        obstacle_speed = 0;
      }

      else {
        client.print("Distance Detected: " + String(obstacle_distance) + " cm" + "\n");
      }
      Serial.println("Distance Detected: " + String(obstacle_distance) + " cm");

      Serial.println("Buggy Speed " + String(bug_speed) + " cm/s");
      client.print("Buggy Speed " + String(bug_speed) + " cm/s" + "\n");
      Serial.println("Obstacle Speed " + String(obstacle_speed) + " cm/s");
      client.print("Obstacle Speed " + String(obstacle_speed) + " cm/s" + "\n");


      //  client.write();
      //  client.write("cm/s");
      // client.write("Obstacle Speed: ");
      // client.write(obstacle_speed);
      // client.write("cm/s");
    }
  }
  if (c == 's') {  //stop button from processing
    //client.write("Stopping!!"); //sends message to proccessing (aka server)
    stop();
  }
}

void speedAdjuster() {

  pid_input = distance_calculator();  //take in distance measured by Ultrasonic

  pid_output = computePID(pid_input);

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

    //test = count / 4.0;

    //count = 0;

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
