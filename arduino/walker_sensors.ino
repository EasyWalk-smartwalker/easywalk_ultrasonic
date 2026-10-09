// =============================================
//   SMART WALKER — Arduino Serial Publisher
//   Reads 6 HC-SR04 sensors, prints distances
//   to Serial for ROS 2 to consume
// =============================================

#define SOUND_SPEED   0.0343
#define PULSE_TIMEOUT 15000UL
#define LOOP_DELAY    20        // ms between cycles

struct Sensor {
  const int   trig;
  const int   echo;
  const char* name;
};

Sensor sensors[6] = {
  {22, 23, "L30"},
  {24, 25, "L15"},
  {26, 27, "CTR"},
  {28, 29, "R15"},
  {30, 31, "R30"},
  {32, 33, "BOT"},
};

void setup() {
  Serial.begin(57600);
  for (int i = 0; i < 6; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }
}

float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, PULSE_TIMEOUT);
  if (duration == 0) return -1.0;
  float dist = (duration * SOUND_SPEED) / 2.0;
  if (dist < 2.0 || dist > 300.0) return -1.0;
  return dist;
}

void loop() {
  for (int i = 0; i < 6; i++) {
    float d = getDistance(sensors[i].trig, sensors[i].echo);
    Serial.print(sensors[i].name);
    Serial.print(":");
    Serial.print(d, 1);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
  delay(LOOP_DELAY);
}
