#include <Arduino.h> // PIO Specific Call Arduino Core
#include <SimpleFOC.h> // Call FOC Library
#include <utils.h> // ESP32 Specific utility that clamps a middle value witin a range of values between a defined min and max bound.

// Driver Logic Pin Defs
#define PIN_UH 21
#define PIN_UL 12
#define PIN_VH 14
#define PIN_VL 10
#define PIN_WH 13
#define PIN_WL 11
// Sensor nCS pin
#define MAQ_SS 4
//Motor Pole Pair Count
#define MOTOR_PP 7 

// Static construct set for Dead Zone Compensation and EWMA correction.
static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
static const float DEAD_ZONE_RAD = 1 * _PI / 180;
static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

// Default Haptic Program Variables
int32_t position = 0;
int32_t num_positions = 360;
float position_with_radians = 2.5*_PI/180.0;;
float haptic_strength_unit = 0.2;
float haptic_endstop_strength_unit = 1.4;
float haptic_dampening_factor = 5500; // min 50 max 7500
float snap_point = 0.545;
float sub_position_unit;

 // Because we don't yet understand EWMA and Dead Zone compensation mechanisms yet, using code borrowed from Scott Bezeks' SmartKnob project.
  float idle_check_velocity_ewma = 0;
  uint32_t last_idle_start = 0;
  uint32_t last_publish = 0;

  const float derivative_lower_strength = haptic_strength_unit * 0.148;
  const float derivative_upper_strength = haptic_strength_unit * 0.008;
  const float derivative_position_width_lower = radians(3); // 3 Degrees represented in radians
  const float derivative_position_width_upper = radians(8); // 8 Dergees represented in radians
  const float raw = derivative_lower_strength + (derivative_upper_strength - derivative_lower_strength)/(derivative_position_width_upper - derivative_position_width_lower)*(position_with_radians - derivative_position_width_lower);



// This code is optimized to works well with MAD2804 motor.

// Call a MagneticSensoSPI driver in SimpleFOC libdep.
// You will need to manually add definiton of the sensor to MagneticSensorSPI.h and MagneticSensorSPI.cpp
// .pio/libdeps/adafruit_feather_esp32s3/SimpleFOC/src/drivers/sensors
// Correct version of the file for SimpleFOC 2.3 can be found in GitHub repository
MagneticSensorSPI sensor = MagneticSensorSPI(MAQ430_SPI, MAQ_SS); 

// Define number of Stator Pole Pairs - Usually this number resides between 7-11 for most gimbal motors
// If you're unsure how many pole pairs your motor has, SimpleFOC calibration would help you estimating PP Count
// Alternatively SimpleFOC has example code "Pole Pair Estimator"
BLDCMotor motor = BLDCMotor(MOTOR_PP);
// BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);

void setup() {

  // SimpleFOC Setup

  // initialise magnetic sensor hardware
  sensor.init();

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // Driver Configuration
  // power supply voltage [V]
  driver.voltage_power_supply = 5;

  // Driver Voltage limit - A way to limit PWM duty cycle
  // For NanoFOC DevKit this value is between 2-5V
  // Do not exceed 5V as you might damage the driver
  driver.voltage_limit = 3;

  // Driver PWM Frequency - Higher the value the smoother operation in theory.
  // If unsure leave commented, Values can range from 8000-50000 (Hz)
  // driver.pwm_frequency = 50000;

  // Initialize Driver
  driver.init();
  // Link the motor and the driver
  motor.linkDriver(&driver);

  // For Haptic Feedback we will be using Torque controller
  // Torque Controller can be controlled by Voltage (Default), Voltage - Estimated Current, DC Current and FOC Current modes.
  
  motor.controller = MotionControlType::torque;

  // If you know your motor phase resistance
  // You can uncomment this section, and you will be operating in "Voltage - Estimated Current" mode.
  // It is much more natural to control haptics with Current rather than Voltage
  // motor.phase_resistance = 4.4; // ex. 2.5 Ohms

  // If you know your motor KV rating (V/RPM) you may instert it here. KV is proportional to target voltage.
  // Most of manufacturers provide KV rating for rated Voltage, It is advised to perform your own KV rating.
  // You can do this by running SimpleFOC KV Rating example code.
  // Please add 10-20% overhead on top of your KV estimation
  //motor.KV_rating = 115;

  // IMPORTANT
  // By introducing Phase Resistance and KV Rating you would need drastically lower your PID values
  // If you decide to introuce Phase R and KV rating start from PID 0,0,0,250,5
  // High probability that BEMF/Inductance spike will damage VBUS diode and LDO if not used correctly


  // Very Basic PID Controller
  // haptic attraction controller - only Proportional + Derivative (Ignore Intergal == 0)
  // P = Proportional, responsible for reactiveness, Values between 0.1 - 1.5 are acceptable for most of motors.
  // D = Derivative, responsible for introducing resistance - similar to viscose friction. Use this to counteract cogging torque or in combination with Proportional. (Keep values low between 0.01 = 0.14)
  // Output_Ramp = Output Derivative limit [V/s] - depending on motor keep it between 100-2500. Non NdFeB drives can use higher values.
  // Limit = Output supply limit [V] - Keep at 3 or less unless you understand what you're doing.
  // [IMPORTANT] If you're not using MAD2804 motor start tuning from PID{0,0,0,100,1}
  // Set motor voltage limit (it is safe to keep it between 2.5-5V, depending on motor is recommended 3V)
  motor.voltage_limit = 3;   // [V]
  motor.PID_velocity.P = 1; // Proportional Gain is further calculated adaptively
  motor.PID_velocity.I = 0; // Intergal Gain remain at "0"
  motor.PID_velocity.D = 0.48; // Derivative Gain is further calculated adaptively as well
  motor.PID_velocity.output_ramp = haptic_dampening_factor; // TODO Output Ramp need to be adaptive 
  motor.PID_velocity.limit = 3; // Velocity limit is further calculated adaptively

  // Velocity Low Pass Filtering
  // Default value is 5ms - You an try different values to see what is best for your motor
  // The lower value the less filtering
  // motor.LPF_velocity.Tf = 0.015;// to set it to 10ms
  // As above but for angle (does not override velocity Low Pass Filtering)
  motor.LPF_angle.Tf = 0.0075;
  
  motor.PID_velocity.D = CLAMP(
    raw,
    min(derivative_lower_strength, derivative_upper_strength),
    max(derivative_lower_strength, derivative_upper_strength)
  );

  // Use monitoring with serial 
  Serial.begin(115200);

  // Comment out if not needed
  motor.useMonitoring(Serial);

  // Initialize motor
  motor.init();
  

  // Align sensor, PP check + Eletrical Zero, if pass start FOC
  motor.initFOC(4.6, Direction::CW);

  Serial.println("Haptic Device Ready :)");
  _delay(500);

  

}

float current_angle_center = -motor.shaft_angle;

void loop() {
 
  // Main FOC algorithm function
  motor.loopFOC();

 // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
  idle_check_velocity_ewma = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
  if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
      last_idle_start = 0;
  } else {
      if (last_idle_start == 0) {
          last_idle_start = millis();
      }
  }
  if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_DELAY_MILLIS && fabsf(motor.shaft_angle - current_angle_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
      current_angle_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_angle_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
  }

float position_to_indent_center = -motor.shaft_angle - current_angle_center;

if (position_to_indent_center > position_with_radians * snap_point && (num_positions <= 0 || position > 0)) {
            current_angle_center += position_with_radians;
            position_to_indent_center -= position_with_radians;
            position--;
            
        } else if (position_to_indent_center < -position_with_radians * snap_point && (num_positions <= 0 || position < num_positions - 1)) {
            current_angle_center -= position_with_radians;
            position_to_indent_center += position_with_radians;
            position++;
           
        }

float dead_zone_adjustment = CLAMP(
  position_to_indent_center,
  fmaxf(-position_with_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
  fminf(position_with_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

bool out_of_range = num_positions > 0 && ((position_to_indent_center > 0 && position == 0) || (position_to_indent_center < 0 && position == num_positions - 1));
motor.PID_velocity.limit = 20;
motor.PID_velocity.P = out_of_range ? haptic_endstop_strength_unit : 3.4;

// Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
if (fabsf(motor.shaft_velocity) > 50) {
    // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
    motor.move(0);
} else {
    float torque = motor.PID_velocity(-position_to_indent_center + dead_zone_adjustment);
    torque = -torque;
    motor.move(torque);
}

// Publish current status to other registered tasks periodically
        if (millis() - last_publish > 10) {
            Serial.print("Current Position: ");
            Serial.print(position + 1);
            Serial.print(" (Derivative: ");
            Serial.print(motor.PID_velocity.D);
            Serial.print(" Proportional: ");
            Serial.print(motor.PID_velocity.P);
            Serial.println(").");
            last_publish = millis();
        }
  
  // Real-time monitoring calls
  // motor.monitor();
  // real-time commander calls
  // command.run();

  // Add microdelay for "Clickiness"
  // Use value between 100-400us. Especially for higher PID or Attractor angle use low values!
  // Do not exceed 1000us
  delayMicroseconds(275);
}