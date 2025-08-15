/* ---------------- Pin map ---------------- */
const unsigned char PIN_SIG    = 8;    // PING))) SIG (PB0 / PCINT0) — shared trig/echo
const unsigned char PIN_DOOR   = 9;    // Door (PB1 / PCINT1) — active LOW with pull-up
const unsigned char PIN_BUZZER = 6;    // Alarm actuator
const unsigned char PIN_LED    = 13;   // Heartbeat LED
const unsigned char PIN_LDR    = A0;   // Photoresistor divider

/* ---------------- Timing & logic constants ---------------- */
const unsigned int  OCR1A_TICKS                = 4999;   // 20 ms @ 16 MHz/64
const unsigned int  DEBOUNCE_TICKS             = 3;      // ~60 ms debounce for door
const unsigned int  HEARTBEAT_PERIOD_TICKS     = 25;     // 500 ms LED blink
const unsigned int  ANALOG_SAMPLE_PERIOD_TICKS = 10;     // 200 ms LDR read
const unsigned int  PING_PERIOD_TICKS          = 8;      // ~160 ms between ultrasonic pings
const unsigned int  ECHO_TIMEOUT_US            = 30000;  // 30 ms (~5 m) max echo
const unsigned int  MIN_CM                     = 3;      // throw away < 3 cm (noise)
const unsigned int  PRINT_PERIOD_TICKS         = 30;     // 300 ms print rate for RANGE
const unsigned int  NEAR_CM                    = 100;    // "near" threshold for alarm logic
const unsigned int  CORRELATION_WINDOW_TICKS   = 500;    // 5 s window to correlate events
const int           LDR_DARK_THRESHOLD         = 500;    // tweak per lighting in sim

/* ---------------- ISR-shared state (volatile) ----------------
   ISRs only flip these flags / save timestamps.
   The main loop reads them atomically and clears them. */
volatile unsigned char  timer_flags   = 0; 
volatile unsigned long  ticks10ms     = 0;
volatile unsigned char  pb_snapshot   = 0;

#define FLAG_SAMPLE_A0      (1 << 0)
#define FLAG_HEARTBEAT      (1 << 1)
#define FLAG_SCHEDULE_PING  (1 << 2)

/* Echo timing captured by PCI ISR on D8 */
volatile unsigned long echo_rise_us = 0;
volatile unsigned long echo_fall_us = 0;
volatile unsigned char echo_state   = 0;   // 0=low, 1=high (tracks current echo level)
volatile unsigned char echo_ready   = 0;   // set when a full pulse (rise→fall) is captured

/* Door change marker set by PCI ISR on D9 (debounced in main loop) */
volatile unsigned char door_change_flag = 0;

/* ---------------- Main (non-ISR) state ---------------- */
bool nearObject = false;      // true when last distance < NEAR_CM 
bool doorOpen   = false;      // true when pushbutton pressed (active LOW)
bool isDark     = false;      // LDR state 
bool alarmOn    = false;      // actuator state

unsigned long nextOkTickDoor   = 0;   // debounce end time
unsigned long lastNearTick     = 0;   // last time nearObject became true
unsigned long lastDoorOpenTick = 0;   // last time door opened

unsigned int  lastAnalog = 0;
bool          ledState = false;

/* ---------------- Small helpers & logging ---------------- */
static inline unsigned long nowTicks(){
  // atomic read of logical time base used in all timestamped logs
  noInterrupts(); unsigned long t = ticks10ms; interrupts(); return t;
}
static void trace_msg(const char* tag, const char* msg){
  // time-stamped trace lines
  unsigned long ms = nowTicks()*10UL;
  Serial.print('['); Serial.print(ms); Serial.print(" ms] ");
  Serial.print(tag); Serial.print(": "); Serial.println(msg);
}
static void trace_val(const char* tag, const char* key, long value, const char* suffix){
  // value trace (used for RANGE and Light)
  unsigned long ms = nowTicks()*10UL;
  Serial.print('['); Serial.print(ms); Serial.print(" ms] ");
  Serial.print(tag); Serial.print(": "); Serial.print(key); Serial.print(value);
  if(suffix && *suffix){ Serial.print(' '); Serial.print(suffix); }
  Serial.println();
}

/* ===================== Setup helpers ===================== */

/* Enable PCI for Port B and unmask PB0 (D8) + PB1 (D9). */
void setupPCI_PortB(){
  pinMode(PIN_SIG,  INPUT);          // SIG idles as input (we briefly drive it when pinging)
  pinMode(PIN_DOOR, INPUT_PULLUP);   // button to GND, so LOW = pressed

  pb_snapshot = PINB;                // baseline snapshot before enabling PCI
  PCICR  |= _BV(PCIE0);              // enable pin-change on Port B
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1); // unmask PB0(D8 SIG), PB1(D9 DOOR)
}

/* Configure Timer1 for 50 Hz CTC. */
void setupTimer1_50Hz(){
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A  = OCR1A_TICKS;              // compare match every 20 ms
  TCCR1B |= _BV(WGM12);              // CTC mode
  TCCR1B |= _BV(CS11)|_BV(CS10);     // prescaler 64
  TIMSK1 |= _BV(OCIE1A);             // enable compare-A interrupt
  interrupts();
}

/* ===================== Periodic (Timer) jobs ===================== */

/* Periodically read the LDR and update the "isDark" flag.
   Prints only on threshold changes or big drifts to keep logs clean.
   Shows timer-driven sensing without blocking. */
void readAnalogIfDue(unsigned char t_flags){
  static unsigned int lastPrinted = 0;
  if (t_flags & FLAG_SAMPLE_A0){
    lastAnalog = analogRead(PIN_LDR);
    bool newDark = (lastAnalog < LDR_DARK_THRESHOLD);
    if (newDark != isDark || (lastAnalog > lastPrinted + 30) || (lastAnalog + 30 < lastPrinted)){
      isDark = newDark; lastPrinted = lastAnalog;
      trace_val("TIMER","Light=", lastAnalog, isDark ? "(DARK)" : "(BRIGHT)");
    }
  }
}

/* Half-second heartbeat of the offboard LED */
void heartbeatIfDue(unsigned char t_flags){
  if (t_flags & FLAG_HEARTBEAT){
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState?HIGH:LOW);
  }
}

/* Emit a trigger pulse on the ultrasonic sensor.
   Done in the main loop to keep interrupts short. */
void sendPingIfDue(unsigned char t_flags){
  if (t_flags & FLAG_SCHEDULE_PING){
    pinMode(PIN_SIG, OUTPUT);
    digitalWrite(PIN_SIG, LOW);  delayMicroseconds(2);
    digitalWrite(PIN_SIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_SIG, LOW);
    pinMode(PIN_SIG, INPUT);
  }
}

/* ===================== PCI-derived jobs ===================== */

/* Door changes are marked in the ISR and debounced here.
   Demonstrates event handling split out of the ISR. */
void handleDoorIfFlag(unsigned long tNow){
  if (!door_change_flag) return;
  noInterrupts(); door_change_flag = 0; interrupts();

  if (tNow >= nextOkTickDoor){
    bool levelLow = (digitalRead(PIN_DOOR) == LOW);  // active LOW with pull-up
    if (levelLow != doorOpen){
      doorOpen = levelLow; nextOkTickDoor = tNow + DEBOUNCE_TICKS;
      if (doorOpen){ lastDoorOpenTick = tNow; trace_msg("PCI","DOOR=OPEN (LOW)"); }
      else         { trace_msg("PCI","DOOR=CLOSED (HIGH)"); }
    }
  }
}

/* When the ISR reports a complete echo pulse, compute distance,
   print at most every 300 ms, and update the "nearObject" state.
   Values under 3 cm are dropped to ignore noise/spikes. */
void handleEchoIfReady(unsigned long tNow){
  if (!echo_ready) return;

  unsigned long rise, fall;
  noInterrupts();  echo_ready = 0;  rise = echo_rise_us;  fall = echo_fall_us;  interrupts();

  unsigned long dur = (fall >= rise) ? (fall - rise) : 0UL;
  if (dur == 0 || dur > ECHO_TIMEOUT_US) return;

  long cm = (long)(dur / 58UL);     // ~58 µs → 1 cm
  if (cm < MIN_CM) return;          // ignore sub-3 cm glitches

  // throttle RANGE prints to keep Serial readable
  static unsigned long lastPrintTicks = 0;
  if (tNow - lastPrintTicks >= PRINT_PERIOD_TICKS) {
    trace_val("RANGE","cm=", cm, "");
    lastPrintTicks = tNow;
  }

  bool newNear = (cm > 0 && cm < NEAR_CM);
  if (newNear != nearObject){
    nearObject = newNear;
    if (nearObject){ lastNearTick = tNow; trace_msg("EVENT","NEAR_OBJECT=TRUE"); }
    else           { trace_msg("EVENT","NEAR_OBJECT=FALSE"); }
  }
}

/* ===================== Think & Act ===================== */

/* State machine:
   Alarm turns ON only when:
     - It's DARK, and
     - "Near object" and "Door open" occur within the 5 s window
       (order independent).
   This demonstrates grouped state & temporal correlation. */
void updateStateMachine(unsigned long tNow){
  bool correlated =
    (nearObject && (tNow - lastDoorOpenTick) <= CORRELATION_WINDOW_TICKS) ||
    (doorOpen   && (tNow - lastNearTick)     <= CORRELATION_WINDOW_TICKS);

  bool newAlarm = (correlated && isDark);
  if (newAlarm != alarmOn){
    alarmOn = newAlarm;
    trace_msg("STATE", alarmOn ?
      "ALARM=ON (near+door within window & dark)" :
      "ALARM=OFF");
  }
}

/* Actuator driver kept tiny and deterministic. */
void driveActuators(){
  digitalWrite(PIN_BUZZER, alarmOn ? HIGH : LOW);
}

/* ===================== Interrupt Service Routines ===================== */

/* PCINT0_vect services both D8 and D9 (same vector).
   We read PINB and compare to a snapshot to learn which pin changed. */
ISR(PCINT0_vect){
  unsigned char cur = PINB;
  unsigned char changed = (unsigned char)((cur ^ pb_snapshot) & (_BV(PB0) | _BV(PB1)));
  unsigned long nowus = micros();

  // D8 (PB0): echo pulse edges for ultrasonic range
  if (changed & _BV(PB0)){
    if (cur & _BV(PB0)){          // rising edge
      echo_state = 1;
      echo_rise_us = nowus;
    } else {                      // falling edge
      if (echo_state){            // only if a rise was seen
        echo_state = 0;
        echo_fall_us = nowus;
        echo_ready = 1;           // full pulse captured
      }
    }
  }

  // D9 (PB1): door changed (debounce handled in main loop)
  if (changed & _BV(PB1)){
    door_change_flag = 1;
  }

  pb_snapshot = cur;              // update snapshot last
}

/* Timer1 compare ISR: */
ISR(TIMER1_COMPA_vect){
  ticks10ms += 2;  // 20 ms tick → +2 of our 10 ms units
  if ((ticks10ms % (ANALOG_SAMPLE_PERIOD_TICKS*2)) == 0) timer_flags |= FLAG_SAMPLE_A0;
  if ((ticks10ms % (HEARTBEAT_PERIOD_TICKS*2))     == 0) timer_flags |= FLAG_HEARTBEAT;
  if ((ticks10ms % (PING_PERIOD_TICKS*2))          == 0) timer_flags |= FLAG_SCHEDULE_PING;
}

/* ===================== Arduino setup / loop ===================== */

/* Initializes IO, Serial, PCI, and Timer. Also prints summary lines */
void setup(){
  pinMode(PIN_LED, OUTPUT);    digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LDR, INPUT);

  Serial.begin(115200);
  Serial.println("\n--- PCI + Timer Sense–Think–Act (PING))) 3-pin) ---");
  Serial.println("Pins: SIG=D8 (Trig+Echo), DOOR=D9, BUZZER=D6, LED=D13, LDR=A0");

  setupPCI_PortB();
  setupTimer1_50Hz();

  // read initial inputs once
  doorOpen  = (digitalRead(PIN_DOOR) == LOW);
  lastAnalog = analogRead(PIN_LDR);
  isDark = (lastAnalog < LDR_DARK_THRESHOLD);

  Serial.print("[INIT] DOOR=");   Serial.print(doorOpen ? "OPEN" : "CLOSED");
  Serial.print(", Light=");       Serial.print(lastAnalog);
  Serial.println(isDark ? " (DARK)" : " (BRIGHT)");
}

/* Main scheduler: */
void loop(){
  noInterrupts();
  unsigned char t_flags  = timer_flags;  timer_flags = 0;
  unsigned long tNow     = nowTicks();
  interrupts();

  // Timer-driven sensing & status
  readAnalogIfDue(t_flags);
  heartbeatIfDue(t_flags);
  sendPingIfDue(t_flags);

  // PCI-driven input handling
  handleDoorIfFlag(tNow);
  handleEchoIfReady(tNow);

  // Decision & actuation
  updateStateMachine(tNow);
  driveActuators();
}
