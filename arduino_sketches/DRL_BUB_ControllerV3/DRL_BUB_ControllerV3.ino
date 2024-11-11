/*
  Board: Generic STM32F4 Series
  Board part number: Generic F411CEUx
  U(S)ART Support: Enabled (generic 'Serial')
  USB Support (if available): CDC (generic 'Serial' supersede U(S)ART)
  USB Speed (if available): Low/Full Speed
  Optimize: Smallest (-Os default)
  Debug symbols and core logs: None
  C Runtime Library: Newlib Nano (default)
  Upload method: STM32CubeProgrammer (DFU)
  
  1. Unplug board
  2. Hold down boot button, plug in
  3. Release boot button
  4. Upload 
*/

#include "SensorFusion.h"
#include <SPI.h>
#include "BMI270.h"
#include <Servo.h>
#include <HardwareSerial.h>

#include <BasicLinearAlgebra.h>
using namespace BLA;

HardwareSerial Serial1(PA3, PA2); // Rx, Tx

Servo ESC[4]; 
int Dir[4] = {-1, 1, 1, 1};
float Kt[4] = {0.0, 0.0, 0.0, 0.0}; // Kalman filter terms for thrust smoothing

// Forward thrust variables (DoF #1)
float thrust_desired  = 0.0;

// Yaw variables            (DoF #2)
float yaw_desired     = 0.0;
float yaw_error       = 0.0;
float yaw_error_dot   = 0.0;
float yaw_error_int   = 0.0;
float Kp_yaw          = 0.0;
float Kd_yaw          = 0.0;
float Ki_yaw          = 0.0;

// Depth variables          (DoF #3)
float depth_desired   = 0.0;
float depth_error     = 0.0;
float depth_error_dot = 0.0;
float depth_error_int = 0.0;
float Kp_depth        = 0.0;
float Kd_depth        = 0.0;
float Ki_depth        = 0.0;

float dive_thrust     = 0.0;

// Roll variables           (DoF #4)
float roll_desired    = 0.0;
float roll_error      = 0.0;
float roll_error_dot  = 0.0;
float roll_error_int  = 0.0;
float Kp_roll         = 0.0;
float Kd_roll         = 0.0;
float Ki_roll         = 0.0;

// axis transform variables
float pitch_raw, roll_raw;
float rot = 45*DEG_TO_RAD;

// control signal variables
float u[4]        = {0.0, 0.0, 0.0, 0.0};
float u_max[4]    = {1000.0, 1000.0, 1000.0, 1000.0};
float F[4]        = {0.0, 0.0, 0.0, 0.0};
float F_min       = 100;
float F_max       = 350;
bool F_active[4]  = {false, false, false, false};

// yaw tracking variables
float last_yaw = 0;
int rotations = 0;
float bias_yaw = 180;

SF fusion;

static const uint8_t MOSI_PIN = PA7;    // resource SPI_MOSI 1 A07
static const uint8_t MISO_PIN = PA6;    // resource SPI_MISO 1 A06
static const uint8_t SCLK_PIN = PA5;    // resource SPI_SCK 1 A05
static const uint8_t CS_PIN   = PA4;    // resource GYRO_CS 1 A04
static const uint8_t ADC_VOLT = PA1;    // ADC for battery voltage
static const uint8_t ADC_CURR = PB0;    // ADC for battery current

static const uint8_t LED_PIN = PC14;    // resource LED 1 C14

// communication with built-in IMU
static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);
static BMI270 imu = BMI270(spi, CS_PIN);

// variables for tracking roll, pitch, yaw
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

// variables for tracking depth
float depth = 0;
float depth_rate = 0;

// variables for control frequency
unsigned int cycle_time_ms = 4;          // 250 Hz
unsigned int last_cycle_time = 0;
unsigned int data_time_ms = 200;          // 5 Hz
unsigned int last_data_time = 0;

// communication variables
char comm_buffer[100];
int comm_index = 0;
bool complete_message = false;
bool comm_in_progress = false;
String comm_vals[10];
unsigned int data_count = 0;
bool record_data = false;
String data;

// internal state variables
float voltage = 0;
float current = 0;
float v_min = 3.3;  // minimum safe battery voltage

// safety variables
bool enabled = true;
unsigned int last_safe_time = 0;
unsigned int safe_time_limit = 36000000;  // 10 hours
float safe_depth_limit = 1000;            // 10 meters
bool safety_enabled = false;

// for printing to serial monitor
bool debug = false;

// control configuration
int thrust_arrangement = 0;
float thrust_adjustment[4] = {1, 1, 1, 1};
float theta = 30.0 * DEG_TO_RAD;
Matrix<4,4> A;

// pre-compute transform entries
float sin_th = sin(rot);
float cos_th = cos(rot);

void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);
  
  spi.begin();
    
  imu.begin();

  ESC[0].attach(PB8);   // motor 1
  ESC[1].attach(PA0);   // motor 2
  ESC[2].attach(PB10);  // motor 3
  ESC[3].attach(PB7);   // motor 4

  for(int i = 0; i < 4; i++)
  {
    ESC[i].writeMicroseconds(1500);
  }
  delay(3000);

  // reset command buffer
  for(int i = 0; i < 100; i++)
  {
    comm_buffer[i] = '\0';
  }

  pinMode(ADC_VOLT, INPUT);
  pinMode(ADC_CURR, INPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, true);
}

void loop() {

  // inertial measurement
  imu.readSensor();  
  ax = imu.getRawAccelX() * 0.0005914; // / 1/16384.0 * 9.8;
  ay = imu.getRawAccelY() * 0.0005914; // / 16384.0 * 9.8;
  az = imu.getRawAccelZ() * 0.0005914; // / 16384.0 * 9.8;
  gx = imu.getRawGyroX() * 0.00106422515; // / 16.4 * DEG_TO_RAD;
  gy = imu.getRawGyroY() * 0.00106422515; // / 16.4 * DEG_TO_RAD;
  gz = imu.getRawGyroZ() * 0.00106422515; // / 16.4 * DEG_TO_RAD;

  // apply mahony filter
  deltat = fusion.deltatUpdate();
  fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);

  // get filtered IMU data
  yaw = fusion.getYaw();
  pitch_raw = fusion.getPitch();
  roll_raw = fusion.getRoll();
  
  // correct yaw to be continuous
  if(last_yaw > 350 && yaw < 10){rotations += 1;}
  else if(last_yaw < 10 && yaw > 350){rotations -= 1;}
  last_yaw = yaw;

  // correct pitch and roll with rigid transform
  roll  = sin_th*pitch_raw + cos_th*roll_raw;
  pitch = cos_th*pitch_raw - sin_th*roll_raw;
  // pitch = cos(theta)*pitch_raw - sin(theta)*roll_raw;

  // compute yaw error terms
  yaw_error = (yaw + rotations*360 - bias_yaw) - yaw_desired;              // proportional
  yaw_error_dot = gz * RAD_TO_DEG;                                         // derivative
  yaw_error_int = yaw_error_int + cycle_time_ms / 1000.0 * yaw_error;      // integral
  if(yaw_error_int > 1000)
  {
    yaw_error_int = 1000;
  }
  else if(yaw_error_int < -1000)
  {
    yaw_error_int = -1000;
  }
  
  // compute roll error terms
  roll_error = roll - roll_desired;                                        // proportional
  roll_error_dot = (sin_th*gx + cos_th*gy) * RAD_TO_DEG;                   // derivative
  roll_error_int = roll_error_int + cycle_time_ms / 1000.0 * roll_error;   // integral
  if(roll_error_int > 1000)
  {
    roll_error_int = 1000;
  }
  else if(roll_error_int < -1000)
  {
    roll_error_int = -1000;
  }
  // compute depth error terms
  depth_error = depth_desired - depth;
  depth_error_dot = depth_rate;
  depth_error_int = depth_error_int + 100.0 / 1000.0 * depth_error; // integral
  
  if(depth_error_int > 1000)
  {
    depth_error_int = 1000;
  }
  else if(depth_error_int < -1000)
  {
    depth_error_int = -1000;
  }

  // compute control inputs

//  u[0] = Kt[0] * u[0] + (1 - Kt[0]) *   (thrust_desired);                                                      // forward control signal (DoF #1)  [unit thrust, 0-400]
//  u[1] = Kt[1] * u[1] + (1 - Kt[1]) *   (Kp_yaw * yaw_error     + Kd_yaw * yaw_error_dot);                     // yaw control signal     (DoF #2)  [unit moment, 0-400]
//  u[2] = Kt[2] * u[2] + (1 - Kt[2]) *   (Kp_depth * depth_error + Kd_depth * depth_error_dot + dive_thrust);   // depth control signal   (DoF #3)  [unit moment, 0-400]
//  u[3] = Kt[3] * u[3] + (1 - Kt[3]) *   (Kp_roll * roll_error   + Kd_roll * roll_error_dot);                   // roll control signal    (DoF #4)  [unit thrust, 0-400]
  
//  u[0] = thrust_desired;                                                      // forward control signal (DoF #1)  [unit thrust, 0-400]
//  u[1] = Kp_yaw * yaw_error     + Kd_yaw * yaw_error_dot;                     // yaw control signal     (DoF #2)  [unit moment, 0-400]
//  u[2] = Kp_depth * depth_error - Kd_depth * depth_error_dot + dive_thrust;   // depth control signal   (DoF #3)  [unit moment, 0-400]
//  u[3] = Kp_roll * roll_error   + Kd_roll * roll_error_dot;                   // roll control signal    (DoF #4)  [unit thrust, 0-400]

  u[0] = thrust_desired;                                                      // forward control signal (DoF #1)  [unit thrust, 0-400]
  u[1] = Kp_yaw * yaw_error     + Kd_yaw * yaw_error_dot                   + Ki_yaw * yaw_error_int;        // yaw control signal     (DoF #2)  [unit moment, 0-1000]
  u[2] = Kp_depth * depth_error - Kd_depth * depth_error_dot + dive_thrust + Ki_depth * depth_error_int;    // depth control signal   (DoF #3)  [unit moment, 0-1000]
  u[3] = Kp_roll * roll_error   + Kd_roll * roll_error_dot                 + Ki_roll * roll_error_int;      // roll control signal    (DoF #4)  [unit thrust, 0-1000]

  // clip control signals
  for (int i = 0; i < 4; i++)
  {
    if(u[i] > u_max[i])
    {
      u[i] = u_max[i];
    }
    else if(u[i] < -u_max[i])
    {
      u[i] = -u_max[i];
    }
  }

  float alpha = 0;
//  if (thrust_arrangement == 1);
//  {
//    float alpha = pitch * DEG_TO_RAD;
//  }
//  
  // compute A matrix
  Matrix<4,4> A = {cos(theta+alpha), -cos(theta-alpha), cos(theta+alpha), -cos(theta-alpha),
      -cos(theta+alpha), cos(theta-alpha), cos(theta+alpha), -cos(theta-alpha),
      sin(theta+alpha), sin(theta-alpha), sin(theta+alpha), sin(theta-alpha),
      -sin(theta+alpha), -sin(theta-alpha), sin(theta+alpha), sin(theta-alpha)};

  // solve for forces
  Matrix<4> u_ = {u[0], u[1], u[2], u[3]};
  auto decomp = LUDecompose(A);
  Matrix<4> F_ = LUSolve(decomp, u_);

  F[0] = F_(0);
  F[1] = F_(1);
  F[2] = F_(2);
  F[3] = F_(3);

  // respect deadband, maximum, and minimum of propulsors
  for(int i = 0; i < 4; i++){
    if(F[i] >= F_min/2 && F[i] < F_min)
    {
      F[i] = F_min;
    }
    else if(F[i] > F_max)
    {
      F[i] = F_max;
    }
    else if(F[i] < F_min/2)
    {
      F[i] = 0;
    }
  }

  // maintain minimum speed to prevent inertia feedback, if desired
  for(int i = 0; i < 4; i++)
  {
    if(F_active[i] && F[i] < F_min)
    {
      F[i] = F_min;
    }
  }

  // make adjustments based on mapping
  for(int i = 0; i < 4; i++)
  {
    F[i] = F[i] * thrust_adjustment[i];
  }

  // write control signals to ESCs
  if(enabled)
  {
    for(int i = 0; i < 4; i++)
    {
      ESC[i].writeMicroseconds(1500 + Dir[i]*F[i]); 
    }
  }
  else
  {
    for(int i = 0; i < 4; i++)
    {
      ESC[i].writeMicroseconds(1500); 
    }
  }

  // measure voltage and current
  voltage = analogRead(ADC_VOLT) * 0.03488 - 0.09;
  current = analogRead(ADC_CURR) * 0.04205 - 0.04;

  // adhere to desired control frequency, handle communications while waiting
  while(millis() - last_cycle_time < cycle_time_ms)
  {
    // check safety limits
    if(safety_enabled && (millis() - last_safe_time >= safe_time_limit || depth > safe_depth_limit))
    {
      // disable robot if outside of acceptable parameters
      enabled = false;
      safety_enabled = false;
    }
    // send data
    else if(record_data && millis() - last_data_time >= data_time_ms)
    {
      // format data
      formatData();

      // send data
      Serial1.println(data);
      last_data_time = millis();
    }
    // interpret any available messages
    else if(complete_message)
    {
      // interpret command
      parseSerialMessage(comm_buffer, comm_index+1, comm_vals);

      // read command values into appropriate memory
      interpretMessage();

      // echo message for recording
      if(record_data && comm_vals[0] != "D")
      {
        Serial1.println(comm_buffer);
      }
    
      // reset command buffer
      for(int i = 0; i < comm_index; i++)
      {
        comm_buffer[i] = '\0';
      }
      comm_index = 0;
      complete_message = false;
    }
    // serial information to read
    else if(Serial1.available())
    {
      comm_buffer[comm_index] = Serial1.read();
      // Serial.print(comm_buffer[comm_index]);
      if(comm_buffer[comm_index] == '[')
      {
        comm_in_progress = true;
      }
      if(comm_buffer[comm_index] == ']')
      {
        complete_message = true;
        comm_in_progress = false;
      }
      if(comm_in_progress)
      {
        comm_index += 1;
      }
    }
    // battery status
    else if(voltage < v_min)
    {
      // disable thrust
      enabled = false;
    }
    // USB serial debugging
    else if(Serial.available())
    {
      // enable debugging messages
      debug = true;
    }
  }
  last_cycle_time = millis();
}

// Function to parse data from radio message
bool parseSerialMessage(char Data[], int size, String Vals[])
{
  String Val;
  int el = 0;
  for(int i = 0; i < size; i++)
  {
    if(Data[i] == ',')
    {
      Vals[el] = Val;
      el++;
      Val = "";
    }
    else if(Data[i] == '[')
    {

    }
    else if(Data[i] == ']')
    {
      Vals[el] = Val;
      return true;
    }
    else
    {
      Val += Data[i];
    }
  }
  return true;
}

void interpretMessage(void)
{
  if(comm_vals[0] == "D")               // Sensor (D)epth
  {
    depth           = comm_vals[1].toFloat();
    depth_rate      = comm_vals[2].toFloat();
  }
  else if(comm_vals[0] == "C")          // DoF (C)ommand
  {
    thrust_desired  = comm_vals[1].toFloat();
    yaw_desired     = comm_vals[2].toFloat();
    depth_desired   = comm_vals[3].toFloat();
    roll_desired    = comm_vals[4].toFloat();
  }
  else if(comm_vals[0] == "P")          // Controller (P)arameters
  {
    Kp_yaw          = comm_vals[1].toFloat();
    Kd_yaw          = comm_vals[2].toFloat();
    Kp_depth        = comm_vals[3].toFloat();
    Kd_depth        = comm_vals[4].toFloat();
    Kp_roll         = comm_vals[5].toFloat();
    Kd_roll         = comm_vals[6].toFloat();
    dive_thrust     = comm_vals[7].toFloat();
  }
  else if(comm_vals[0] == "Y")     // Reset (Y)aw estimate
  {
    rotations = 0;
    bias_yaw = last_yaw;
  }
  else if(comm_vals[0] == "E")     // (E)nable thrust
  {
    enabled = true;
  }
  else if(comm_vals[0] == "H")     // (H)alt thrust
  {
    enabled = false;
  }
  else if(comm_vals[0] == "R")     // (R)ecord data
  {
    record_data = true;
  }
  else if(comm_vals[0] == "S")     // (S)top Recording
  {
    record_data = false;
  }
  else if(comm_vals[0] == "B")     // (B)ind Propulsor Directions
  {
    Dir[0] = int(comm_vals[1].toFloat());
    Dir[1] = int(comm_vals[2].toFloat());
    Dir[2] = int(comm_vals[3].toFloat());
    Dir[3] = int(comm_vals[4].toFloat());
  }
  else if(comm_vals[0] == "F")     // Change Control (F)requency
  {
    cycle_time_ms = int(1000 / comm_vals[1].toFloat());
    // maximum control frequency of 500 Hz
    if(cycle_time_ms < 2)
    {
      cycle_time_ms = 2;
    }
  }
  else if(comm_vals[0] == "A")     // Set Propulsors to Remain (A)ctive
  {
    F_active[0] = bool(int(comm_vals[1].toFloat()));
    F_active[1] = bool(int(comm_vals[2].toFloat()));
    F_active[2] = bool(int(comm_vals[3].toFloat()));
    F_active[3] = bool(int(comm_vals[4].toFloat()));
  }
  else if(comm_vals[0] == "L")     // Set Safety (L)imits
  {
    last_safe_time = millis();
    safe_time_limit = comm_vals[1].toInt() * 1000;
    safe_depth_limit = comm_vals[2].toFloat();
    safety_enabled = true;
  }
  else if(comm_vals[0] == "M")     // Set Thrust (M)in / Max microseconds
  {
    F_min = comm_vals[1].toFloat();
    F_max = comm_vals[2].toFloat();

    if(F_min < 0)
    {
      F_min = 0;
    }
    if(F_max > 400)
    {
      F_max = 400;
    }
  }
  else if(comm_vals[0] == "K")    // 1D (K)alman filter adjust for thrust levels
  {
    Kt[0] = comm_vals[1].toFloat();
    Kt[1] = comm_vals[2].toFloat();
    Kt[2] = comm_vals[3].toFloat();
    Kt[3] = comm_vals[4].toFloat();

    for(int i = 0; i < 4; i++)
    {
      if(Kt[i] > 1.0)
      {
        Kt[i] = 1.0;
      }
      else if(Kt[i] < 0.0)
      {
        Kt[i] = 0.0;
      }
    }
  }
  else if(comm_vals[0] == "T")    // Change (T)hrust arrangement
  {
    thrust_arrangement = (int)comm_vals[1].toFloat();
  }
  else if(comm_vals[0] == "J")    // Set thrust ad(j)ustment
  {
    thrust_adjustment[0] = comm_vals[1].toFloat();
    thrust_adjustment[1] = comm_vals[2].toFloat();
    thrust_adjustment[2] = comm_vals[3].toFloat();
    thrust_adjustment[3] = comm_vals[4].toFloat();
  }
  else if(comm_vals[0] == "U")    // Set (U) maximums
  {
    u_max[0] = comm_vals[1].toFloat();
    u_max[1] = comm_vals[2].toFloat();
    u_max[2] = comm_vals[3].toFloat();
    u_max[3] = comm_vals[4].toFloat();
  }
  else if(comm_vals[0] == "I")    // Set (I)ntegral terms
  {
    Ki_yaw = comm_vals[1].toFloat();
    Ki_depth = comm_vals[1].toFloat();
    Ki_roll = comm_vals[1].toFloat();

    yaw_error_int = 0;
    depth_error_int = 0;
    roll_error_int = 0;
  }
}
void formatData(void)
{
  // time, desired thrust, desired yaw, desired depth, desired roll, T1, T2, T3, T4, depth, roll, pitch, yaw, voltage, current
  data = "";
  // data += String(data_count);
  // data += ",";
  data += String((float)millis() / 1000.0, 2);
  data += ",";
  data += String(thrust_desired,1);
  data += ",";
  data += String(yaw_desired,1);
  data += ",";
  data += String(depth_desired,1);
  data += ",";
  data += String(roll_desired,1);
  data += ",";
  for(int i = 0; i < 4; i++)
  {
    data += String(u[i],0);
    data += ",";
  }
  for(int i = 0; i < 4; i++)
  {
    data += String(F[i],0);
    data += ",";
  }
  data += String(depth,1);
  data += ",";
  data += String(roll,1);
  data += ",";
  data += String(pitch,1);
  data += ",";
  data += String(yaw + rotations*360 - bias_yaw,1);
  data += ",";
  data += String(voltage,2);
  data += ",";
  data += String(current,2);
  data_count += 1;
}
