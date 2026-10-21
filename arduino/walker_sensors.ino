// =====================================================================
//   SMART WALKER -- Arduino (Mega) firmware for 6 HC-SR04 sensors
//
//   The firmware does NOT convert to distance: it streams the raw
//   time of flight (microseconds) of every ping, with its own timestamp,
//   so that speed-of-sound compensation and per-sensor calibration are
//   applied in ROS (serial_publisher.py) and can be redone offline.
//
//   Output, one line per ping (ASCII, 115200 baud):
//     R,<cycle>,<sensor>,<t_trig_us>,<tof_us>,<status>
//       cycle     : cycle counter (uint32, wraps), lets ROS detect lost lines
//       sensor    : index 0..5 = L30, L15, CTR, R15, R30, BOT
//       t_trig_us : micros() at the trigger pulse (uint32, wraps ~71 min)
//       tof_us    : echo pulse length in us (round-trip time); 0 if no echo
//       status    : 0 = echo received, 1 = no echo before timeout
//   and one line at the end of each cycle:
//     C,<cycle>,<t_us>
//
//   Commands (one per line, terminated by '\n'):
//     M R          ring mode: fire all sensors of the order list (default)
//     M S <i>      single mode: fire only sensor i (calibration)
//     O <i,i,...>  firing order, e.g. "O 0,3,2,1,4,5"
//     D <ms>       pause between two consecutive pings (default 0)
//     P <ms>       pause at the end of each cycle (default 20)
//     T <us>       echo timeout (default 15000 us, ~2.5 m)
//     Q <ms>       minimum period between two pings of the same sensor
//                  (default 60 ms, as recommended by the HC-SR04 datasheet)
//     ?            print current configuration as "# ..." lines
//   Lines starting with '#' are informational and ignored by ROS.
// =====================================================================

const uint8_t N_SENSORS = 6;

struct Sensor {
  const uint8_t trig;
  const uint8_t echo;
  const char*   name;
};

Sensor sensors[N_SENSORS] = {
  {22, 23, "L30"},
  {24, 25, "L15"},
  {26, 27, "CTR"},
  {28, 29, "R15"},
  {30, 31, "R30"},
  {32, 33, "BOT"},
};

// ---- runtime configuration (changeable over serial) -----------------
// Default order alternates sides so that two consecutive pings are not
// fired by neighbouring sensors (reduces crosstalk).
uint8_t  order[N_SENSORS]  = {0, 3, 2, 1, 4, 5};   // L30, R15, CTR, L15, R30, BOT
uint8_t  orderLen          = N_SENSORS;
bool     singleMode        = false;
uint8_t  singleSensor      = 2;                    // CTR
uint16_t interPingDelayMs  = 0;
uint16_t cycleDelayMs      = 20;
uint32_t echoTimeoutUs     = 15000UL;
// Minimum time between two pings of the SAME sensor (HC-SR04 datasheet:
// >= 60 ms), so that late echoes of the previous ping are not read as new.
uint16_t minSensorPeriodMs = 60;

uint32_t cycle = 0;
uint32_t lastTrigUs[N_SENSORS] = {0};
bool     triggeredOnce[N_SENSORS] = {false};
char     cmdBuf[48];
uint8_t  cmdLen = 0;

// ---------------------------------------------------------------------
void printConfig() {
  Serial.print(F("# mode="));       Serial.println(singleMode ? F("single") : F("ring"));
  Serial.print(F("# single="));     Serial.println(singleSensor);
  Serial.print(F("# order="));
  for (uint8_t i = 0; i < orderLen; i++) {
    Serial.print(order[i]);
    if (i + 1 < orderLen) Serial.print(',');
  }
  Serial.println();
  Serial.print(F("# inter_ping_ms=")); Serial.println(interPingDelayMs);
  Serial.print(F("# cycle_ms="));      Serial.println(cycleDelayMs);
  Serial.print(F("# timeout_us="));    Serial.println(echoTimeoutUs);
  Serial.print(F("# min_sensor_period_ms=")); Serial.println(minSensorPeriodMs);
}

void handleCommand(char* cmd) {
  if (cmd[0] == '?') {
    printConfig();
    return;
  }
  char* arg = cmd + 1;
  while (*arg == ' ') arg++;

  switch (cmd[0]) {
    case 'M':
      if (arg[0] == 'R') {
        singleMode = false;
      } else if (arg[0] == 'S') {
        int i = atoi(arg + 1);
        if (i >= 0 && i < N_SENSORS) { singleMode = true; singleSensor = i; }
        else { Serial.println(F("# err sensor index")); return; }
      }
      break;
    case 'O': {
      uint8_t newOrder[N_SENSORS];
      uint8_t n = 0;
      char* tok = strtok(arg, ",");
      while (tok != NULL && n < N_SENSORS) {
        int i = atoi(tok);
        if (i < 0 || i >= N_SENSORS) { Serial.println(F("# err order")); return; }
        newOrder[n++] = i;
        tok = strtok(NULL, ",");
      }
      if (n == 0) { Serial.println(F("# err order")); return; }
      for (uint8_t k = 0; k < n; k++) order[k] = newOrder[k];
      orderLen = n;
      break;
    }
    case 'D': interPingDelayMs = atoi(arg); break;
    case 'P': cycleDelayMs     = atoi(arg); break;
    case 'T': echoTimeoutUs    = strtoul(arg, NULL, 10); break;
    case 'Q': minSensorPeriodMs = atoi(arg); break;
    default:
      Serial.println(F("# err unknown command"));
      return;
  }
  Serial.println(F("# ok"));
  printConfig();
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) handleCommand(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// Fires one sensor and prints its "R" line.
void ping(uint8_t i) {
  // Respect the minimum period of this sensor (unsigned arithmetic is
  // wrap-safe for micros()).
  if (triggeredOnce[i]) {
    uint32_t elapsed = micros() - lastTrigUs[i];
    uint32_t minPeriodUs = (uint32_t)minSensorPeriodMs * 1000UL;
    if (elapsed < minPeriodUs) {
      uint32_t waitUs = minPeriodUs - elapsed;
      delay(waitUs / 1000UL);
      delayMicroseconds(waitUs % 1000UL);
    }
  }

  digitalWrite(sensors[i].trig, LOW);
  delayMicroseconds(2);
  uint32_t tTrig = micros();
  lastTrigUs[i] = tTrig;
  triggeredOnce[i] = true;
  digitalWrite(sensors[i].trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensors[i].trig, LOW);

  // pulseIn returns 0 if no complete pulse is seen within the timeout.
  uint32_t tof = pulseIn(sensors[i].echo, HIGH, echoTimeoutUs);
  uint8_t status = (tof == 0) ? 1 : 0;

  Serial.print(F("R,"));
  Serial.print(cycle);    Serial.print(',');
  Serial.print(i);        Serial.print(',');
  Serial.print(tTrig);    Serial.print(',');
  Serial.print(tof);      Serial.print(',');
  Serial.println(status);
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < N_SENSORS; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    digitalWrite(sensors[i].trig, LOW);
    pinMode(sensors[i].echo, INPUT);
  }
  Serial.println(F("# easywalk_hcsr04 fw 2.0"));
  printConfig();
}

void loop() {
  pollSerial();

  if (singleMode) {
    ping(singleSensor);
  } else {
    for (uint8_t k = 0; k < orderLen; k++) {
      ping(order[k]);
      if (interPingDelayMs > 0 && k + 1 < orderLen) delay(interPingDelayMs);
    }
  }

  Serial.print(F("C,"));
  Serial.print(cycle);
  Serial.print(',');
  Serial.println(micros());
  cycle++;

  delay(cycleDelayMs);
}
