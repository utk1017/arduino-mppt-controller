#include <Wire.h>
#include <Adafruit_INA219.h>
#include <LiquidCrystal_I2C.h>


Adafruit_INA219 ina219;

LiquidCrystal_I2C lcd(0x27, 16, 2);

const float R1 = 20000.0;
const float R2 = 4700.0;
const float DIVIDER_RATIO = (R1 + R2) / R2; 

const float ADC_REF   = 5.0;
const float ADC_STEPS = 1023.0;

const float VOC_PANEL    = 23.0;  
const float VMPP_PANEL   = 17.2;  
const float IMPP_PANEL   = 0.29;  
const float PMPP_PANEL   = 4.99;  


const float BATT_NOMINAL = 11.1;
const float BATT_FULL    = 12.6;  
const float BATT_LOW     = 9.0;   
const int PWM_PIN  = 9;
// Duty cycle 0–255 mapped to Timer1 range 0–319
// D = Vbat / Vmpp = 11.1 / 17.2 = 0.645 → start at 64.5%
const int DUTY_START = 164;  // 164/255 = 64.3% ≈ 11.1V/17.2V
const int DUTY_MIN   = 51;   // 20% floor — protects inductor
const int DUTY_MAX   = 230;  // 90% ceiling — keeps diode time

const int          PERTURB_STEP   = 2;
const unsigned long MPPT_INTERVAL = 100; // ms between steps

// Timing 
const unsigned long LOG_INTERVAL     = 250;  // UART to ESP32 (ms)
const unsigned long DISPLAY_INTERVAL = 600;  // LCD refresh (ms)
const unsigned long UART_TIMEOUT     = 300;  // ESP32 response wait (ms)


float panelVoltage  = 0.0;  // V
float panelCurrent  = 0.0;  // A
float panelPower    = 0.0;  // W
float battVoltage   = 0.0;  // V (estimated from duty + panel V)

int   dutyCycle     = DUTY_START;
float prevPower     = 0.0;
int   prevDuty      = DUTY_START;
int   direction     = 1;     // +1 increase duty, -1 decrease

bool  charging = true;  // false when battery full or low panel

unsigned long lastMpptTime   = 0;
unsigned long lastLogTime   = 0;
unsigned long lastDisplayTime = 0;
bool headerPrinted = false;


void setup() {
  // Serial for CSV logging to  PC for debugging
  Serial.begin(9600);

  // INA219
  if (!ina219.begin()) {
    while (1) {
      if (Serial) Serial.println("ERROR: INA219 not found. Check wiring.");
      delay(1000);
    }
  }
  
  // This sets internal gain for max 400mA and 16V bus voltage
  ina219.setCalibration_16V_400mA();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("MPPT Controller");
  lcd.setCursor(0, 1);
  lcd.print("Initialising...");
  delay(1200);
  lcd.clear();

  // PWM 
  setupPWM();

  // Print CSV header ( PC logging)
  Serial.println("time_ms,volt_V,curr_A,power_W,duty_pct,batt_est_V,source");
}


  pinMode(PWM_PIN, OUTPUT);
  // Fast PWM, no prescaler ,16MHz / 320 = 50kHz
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1   = 319;
  OCR1A  = map(DUTY_START, 0, 255, 0, 319);
}

void setDuty(int d) {
  dutyCycle = constrain(d, DUTY_MIN, DUTY_MAX);
  OCR1A = map(dutyCycle, 0, 255, 0, 319);
}

float getDutyPct() {
  return (dutyCycle / 255.0) * 100.0;
}

// Estimate battery voltage: Vbat ≈ D × Vout (ideal buck)
float estimateBattV() {
  return (dutyCycle / 255.0) * panelVoltage;
}


void readSensors() {
  // Panel voltage via divider on A0
  int raw = analogRead(A0);
  float adcV = (raw / ADC_STEPS) * ADC_REF;
  panelVoltage = adcV * DIVIDER_RATIO;

  // Panel current via INA219 + shunt correction
  float rawmA = ina219.getCurrent_mA();
  panelCurrent = max(0.0, rawmA / 1000.0);

  // Power
  panelPower = panelVoltage * panelCurrent;

  // Estimated battery voltage
  battVoltage = estimateBattV();
}


  unsigned long now = millis();
  if (now - lastMpptTime < MPPT_INTERVAL) return;
  lastMpptTime = now;

  readSensors();
  if (battVoltage >= BATT_FULL) {
    setDuty(DUTY_MIN);
    charging = false;
    return;
  }
  if (panelVoltage < 10.0) {
    setDuty(DUTY_MIN);
    charging = false;
    return;
  }
  charging = true;

  // If ESP32 has sent a TinyML duty suggestion, use it
  // ESP32 prediction overrides P&O when available
  if (esp32Available && esp32Duty > 0) {
    int mlDuty = (int)((esp32Duty / 100.0) * 255.0);
    setDuty(mlDuty);
    esp32Available = false;  
    prevPower = panelPower;
    prevDuty  = dutyCycle;
    return;
  }

  // Standard P&O
  float dP = panelPower - prevPower;
  int   dD = dutyCycle  - prevDuty;

  if (abs(dP) > 0.005) {  
    if (dP > 0) {
      // Power went up , keep same direction
      direction = (dD >= 0) ? 1 : -1;
    } else {
      // Power went down , reverse direction
      direction = (dD >= 0) ? -1 : 1;
    }
  }
  // If dP ≈ 0 we're near MPP 

  prevPower = panelPower;
  prevDuty  = dutyCycle;
  setDuty(dutyCycle + direction * PERTURB_STEP);
}


void uartLog() {
  unsigned long now = millis();
  if (now - lastLogTime < LOG_INTERVAL) return;
  lastLogTime = now;

  // Send CSV row to ESP32
  // Format: time_ms,voltage_V,current_A,power_W,duty_pct,batt_est_V,source
  if (!headerPrinted) {
    Serial.println("time_ms,volt_V,curr_A,power_W,duty_pct,batt_V,source");
    headerPrinted = true;
  }
  Serial.print(now);               Serial.print(",");
  Serial.print(panelVoltage, 3);   Serial.print(",");
  Serial.print(panelCurrent, 4);   Serial.print(",");
  Serial.print(panelPower,   3);   Serial.print(",");
  Serial.print(getDutyPct(), 2);   Serial.print(",");
  Serial.print(battVoltage,  2);   Serial.print(",");
  Serial.println(esp32Available ? "ML" : "PO");
}

void uartReceive() {
  // Check if ESP32 sent a duty cycle recommendation
  // ESP32 sends a single float as string terminated by newline
  
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0 && line.length() < 8) {
      float d = line.toFloat();
      if (d >= 20.0 && d <= 90.0) { 
        esp32Duty      = d;
        esp32Available = true;
      }
    }
  }
}


// Layout:
// Row 0: "V:17.2 I:0.29A"
// Row 1: "P:4.99W D:68.5%"
// When battery full:
// Row 0: "V:xx.x  FULL!  "
// Row 1: "Battery charged "
void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayTime < DISPLAY_INTERVAL) return;
  lastDisplayTime = now;

  lcd.clear();

  if (!charging && battVoltage >= BATT_FULL) {
    // Battery full screen
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(panelVoltage, 1);
    lcd.print("  FULL!");
    lcd.setCursor(0, 1);
    lcd.print("Battery charged!");
    return;
  }

  if (!charging && panelVoltage < 10.0) {
    // No panel / night screen
    lcd.setCursor(0, 0);
    lcd.print("No panel signal ");
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(panelVoltage, 1);
    lcd.print("  waiting...");
    return;
  }

  
  lcd.setCursor(0, 0);
  lcd.print("V:");
  String vStr = String(panelVoltage, 1);
  lcd.print(vStr);
  lcd.print(" I:");
  String iStr = String(panelCurrent, 2);
  lcd.print(iStr);
  lcd.print("A");

  // Row 1
  lcd.setCursor(0, 1);
  lcd.print("P:");
  String pStr = String(panelPower, 2);
  lcd.print(pStr);
  lcd.print("W D:");
  // Print duty cycle as integer percent
  lcd.print((int)getDutyPct());
  lcd.print("%");

  // Show ML indicator if ESP32 is controlling
  if (esp32Available) {
    lcd.setCursor(14, 1);
    lcd.print("ML");
  }
}


void loop() {
  uartReceive();    // check for ESP32 duty suggestion (non-blocking)
  mpptStep();       // runs every 100ms — P&O or ML duty
  uartLog();        // runs every 250ms — CSV to ESP32
  updateDisplay();  // runs every 600ms — LCD refresh
}
