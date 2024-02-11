#include <WiFiNINA.h>

char ssid[] = "VM894C8E2";
char pass[] = "hysuMdbyu2kV";

const int LEYE = 9;
const int REYE = 10;
const int US_TRIG = 15;
const int US_ECHO = 18;

#define A1 6
#define A2 4
#define A3 3
#define A4 5
#define ENA A0  //enables right
#define ENB A7  //enables left

WiFiClient client;
WiFiServer server(5200);
//192,168.4.1 arduino

void setup() {
  Serial.begin(9600);

  WiFi.begin(ssid, pass);
  IPAddress ip = WiFi.localIP();
  server.begin();

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
}

void forwards() {
  int valA = analogRead(ENA);
  int valB = analogRead(ENB);
  analogWrite(ENA, valA / 4);
  analogWrite(ENB, valB / 4);
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A4, LOW);
}

void stop() {
  int valA = analogRead(ENA);
  int valB = analogRead(ENB);
  analogWrite(ENA, valA / 4);
  analogWrite(ENB, valB / 4);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW);
  Serial.println("Stop ");
}

void loop() {
  forwards();
  Serial.print("Let me Go!!\n");


  //recieving info from processing
  char c = client.read();
  if (c == 'g') {
    forwards();
  } else if (c == 's') {
    stop();
  }
  Serial.write(c);
  delay(500);

  //sending info to processing
  server.print("Hello!");
  delay(500);
}