#include "SettingsLowLevel.h"
#include "SteeringController.h"
#ifndef TESTING
#include <Arduino.h>
#include "PID_v1.h"
#endif

SteeringController::SteeringController():
	steerPID(&steerAngleUS, &PIDSteeringOutput_us, &desiredTurn_us, proportional_steering, integral_steering, derivative_steering, DIRECT)
{
	//Hacky fix for not burning servo circuit
  pinMode(STEER_ON, OUTPUT);

  steerPID.SetOutputLimits(MIN_TURN_US, MAX_TURN_US);
  steerPID.SetSampleTime(PID_CALCULATE_TIME);
  steerPID.SetMode(AUTOMATIC);

  Steer_Servo.attach(STEER_OUT_PIN);
  delay(1);
  Steer_Servo.write(90);
	if(DEBUG){
    Serial.println("Steering Setup Complete");
	}
}

SteeringController::~SteeringController()
{
}


int32_t SteeringController::update(int32_t desiredAngle) {
  desiredAngle=map(desiredAngle,MIN_TURN_Mdegrees,MAX_TURN_Mdegrees, MIN_TURN_US,MAX_TURN_US);
  
  if (USE_PIDS) {
    SteeringPID(desiredAngle);
  }
  else {
    engageSteering(desiredAngle);
  }
   //delay(1);
  steerAngleUS = computeAngleRight();
  return map(currentSteeringUS, MIN_TURN_US,MAX_TURN_US,MIN_TURN_Mdegrees,MAX_TURN_Mdegrees);
  //return desiredAngle;
}

//Private
void SteeringController::SteeringPID(int32_t input) {
  desiredTurn_us = input;
	
	steerPID.Compute(); 
	if (PIDSteeringOutput_us != currentSteeringUS) {
		Steer_Servo.writeMicroseconds(PIDSteeringOutput_us);
		currentSteeringUS = PIDSteeringOutput_us;
	}
}

void SteeringController::engageSteering(int32_t input) {
	if (input > MAX_TURN_US)
   input = MAX_TURN_US;
  else if (input < MIN_TURN_US)
    input = MIN_TURN_US;
	if (currentSteeringUS != input) {
		if (DEBUG) {
			Serial.print("MAP Steering: ");
			Serial.println(input);
		}
		currentSteeringUS = input;
	}
		Steer_Servo.writeMicroseconds(input);
   delay(1);
}

int32_t SteeringController::computeAngleLeft() {
	int32_t val = analogRead(AngleSensorLeft);
	val = map(val, Left_Read_at_MIN_TURN, Left_Read_at_MAX_TURN, MIN_TURN_US, MAX_TURN_US);
  /*if(DEBUG){
    Serial.print("Left sensor: ");
    Serial.println(val);
  } */
  return val;
}

int32_t SteeringController::computeAngleRight() {
	int32_t val = analogRead(AngleSensorRight);
	val = map(val, Right_Read_at_MIN_TURN, Right_Read_at_MAX_TURN, MIN_TURN_US, MAX_TURN_US);
/* if(DEBUG){
 Serial.print("Right sensor: ");
 Serial.println(val);
 } */
	return val;
}
