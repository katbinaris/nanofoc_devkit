#include <Arduino.h>
#include <SimpleFOC.h> // Call FOC Library

#define PIN_UH 21
#define PIN_UL 12
#define PIN_VH 14
#define PIN_VL 10
#define PIN_WH 13
#define PIN_WL 11

#define MAQ_SS 4

#define MOTOR_PP 7 // Pole Pairs

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

  // If you know your motor phase resistance (Easily to measure with Multimeter)
  // You can uncomment this section, and you will be operating in "Voltage - Estimated Current" mode.
  // It is much more natural to control haptics with Current rather than Voltage
  // Very high risk of damaging Schottky diode and LDO
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

  // Set motor voltage limit (it is safe to keep it between 2.5-5V, depending on motor is recommended 3V)
  motor.voltage_limit = 3;   // [V]
  
  // Use monitoring with serial 
  Serial.begin(115200);

  // Comment out if not needed
  motor.useMonitoring(Serial);

  // Velocity Low Pass Filtering
  // Default value is 5ms - You an try different values to see what is best for your motor
  // The lower value the less filtering
  motor.LPF_velocity.Tf = 0.01;// to set it to 10ms

  // As above but for angle (does not override velocity Low Pass Filtering)
  motor.LPF_angle.Tf = 0.01;

  // Initialize motor
  motor.init();

  // align sensor, PP check + Eletrical Zero. If pass start FOC
  motor.initFOC();

  Serial.println("Device Ready.");
  _delay(1000);

}


// Very Basic Haptic PID Controller
// haptic attraction controller - only Proportional + Derivative (Ignore Intergal == 0)
// P = Proportional, responsible for reactiveness, Values between 0.1 - 1.5 are acceptable for most of motors.
// D = Derivative, responsible for introducing resistance - similar to viscose friction. Use this to counteract cogging torque or in combination with Proportional. (Keep values low between 0.01 = 0.14)
// Output_Ramp = Output Derivative limit [V/s] - depending on motor keep it between 100-2500. Non NdFeB drives can use higher values.
// Limit = Output supply limit [V] - Keep at 3 or less unless you understand what you're doing.
// [IMPORTANT] If you're not using MAD2804 motor start tuning from PID{0,0,0,100,1}
PIDController PD_haptic{.P=.50,.I=0,.D=0.04,.output_ramp=2550,.limit=3};



// Attractor angle variable
float attract_angle = 0;

// Distance between attraction points - Indentation Amount
// You want to edit this line if you want to increase/decrease amount of steps then correct PIDController above.
float attractor_distance = 3.75*_PI/180.0;




float findAttractor(float current_angle){
  return round(current_angle/attractor_distance)*attractor_distance;
}

void loop() {
 

  // Main FOC algorithm function
  motor.loopFOC();

  // Motion haptic control function
  motor.move(PD_haptic(attract_angle - motor.shaft_angle));

  // Calculate the attractor
  attract_angle = findAttractor(motor.shaft_angle);

  
  // Real-time monitoring calls
  // motor.monitor();
  // real-time commander calls
  // command.run();

  // Add microdelay for "Clickiness"
  // Use value between 100-400us. Especially for higher PID or Attractor angle use low values!
  // Do not exceed 1000us
  delayMicroseconds(375);
}
