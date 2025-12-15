/*
 * PROFESSIONAL MANUAL I2C DRIVER (Simulated Open-Drain)
 * Device: MPU6050 on STM32 Nucleo
 * Output: Pitch & Roll (Degrees) for Serial Plotter
 */

#include <math.h> // Required for atan2()

// --- CONFIGURATION ---
// STM32 Nucleo standard I2C pins
#define SCL_PIN D15  // PB8
#define SDA_PIN D14  // PB9
#define MPU_ADDR 0x68

// --- LOW LEVEL DRIVER (Electrical Standard) ---

void i2c_delay() {
  delayMicroseconds(5); // Adjust speed (5us = ~100kHz)
}

// STANDARD: To send "High", we set pin to INPUT.
// The external 4.7k resistor pulls the line up.
void write_high(int pin) {
  pinMode(pin, INPUT); 
}

// STANDARD: To send "Low", we set pin to OUTPUT and drive Low.
// The chip acts as a sink to Ground.
void write_low(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void i2c_init() {
  // Idle state: Let resistors pull lines High
  write_high(SCL_PIN);
  write_high(SDA_PIN);
}

void i2c_start() {
  // Start Condition: SDA drops while SCL is High
  write_high(SDA_PIN);
  write_high(SCL_PIN);
  i2c_delay();
  write_low(SDA_PIN);
  i2c_delay();
  write_low(SCL_PIN);
}

void i2c_stop() {
  // Stop Condition: SDA rises while SCL is High
  write_low(SDA_PIN);
  i2c_delay();
  write_high(SCL_PIN);
  i2c_delay();
  write_high(SDA_PIN);
  i2c_delay();
}

bool i2c_write(uint8_t data) {
  for (int i = 0; i < 8; i++) {
    // MSB First
    if (data & 0x80) write_high(SDA_PIN);
    else write_low(SDA_PIN);
    
    data <<= 1;
    i2c_delay();
    write_high(SCL_PIN); // Clock High
    i2c_delay();
    write_low(SCL_PIN);  // Clock Low
  }
  
  // Read ACK (Pulse clock and check SDA)
  write_high(SDA_PIN); // Release line for slave
  write_high(SCL_PIN);
  i2c_delay();
  bool ack = !digitalRead(SDA_PIN); // Logic 0 = ACK
  write_low(SCL_PIN);
  return ack;
}

uint8_t i2c_read(bool send_ack) {
  write_high(SDA_PIN); // Ensure line is released
  uint8_t data = 0;
  
  for (int i = 0; i < 8; i++) {
    write_high(SCL_PIN); // Clock High
    i2c_delay();
    data <<= 1;
    if (digitalRead(SDA_PIN)) data |= 1; // Read bit
    write_low(SCL_PIN);  // Clock Low
    i2c_delay();
  }
  
  // Send ACK (Low) or NACK (High)
  if (send_ack) write_low(SDA_PIN);
  else write_high(SDA_PIN);
  
  write_high(SCL_PIN);
  i2c_delay();
  write_low(SCL_PIN);
  
  write_high(SDA_PIN); // Release line
  return data;
}

// --- MAIN SETUP ---
void setup() {
  Serial.begin(115200);
  i2c_init();
  
  Serial.println("--- STM32 MPU6050 Initialized ---");

  // Wake up MPU6050
  i2c_start();
  if (i2c_write(MPU_ADDR << 1)) { // Write Address
    i2c_write(0x6B); // Power Management Register
    i2c_write(0x00); // Wake up command
    i2c_stop();
    Serial.println("Sensor Online. Open Serial Plotter!");
  } else {
    i2c_stop();
    Serial.println("ERROR: Sensor not found (Check Pull-up Resistors)");
  }
}

// --- MAIN LOOP ---
void loop() {
  // 1. Setup Pointer to Accel X (0x3B)
  i2c_start();
  i2c_write(MPU_ADDR << 1); 
  i2c_write(0x3B);
  i2c_stop(); 

  // 2. Read 6 Bytes (Accel X, Y, Z)
  i2c_start();
  i2c_write((MPU_ADDR << 1) | 1); // Read Address
  
  int16_t AcX = (i2c_read(true) << 8) | i2c_read(true);
  int16_t AcY = (i2c_read(true) << 8) | i2c_read(true);
  int16_t AcZ = (i2c_read(true) << 8) | i2c_read(false); // NACK last byte
  i2c_stop();

  // 3. Math: Convert Raw Data to Gravity & Angles
  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;

  // Calculate Pitch & Roll (Result in Degrees)
  // 57.2958 is the conversion factor for Radians to Degrees (180/PI)
  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 57.2958;
  float roll  = atan2(-ax, az) * 57.2958;

  // 4. Visualization Output
  Serial.print("Pitch:"); Serial.print(pitch);
  Serial.print(",");
  Serial.print("Roll:"); Serial.println(roll);

  delay(50); // Keep update rate smooth
}