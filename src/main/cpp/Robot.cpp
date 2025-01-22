// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
using namespace ctre::phoenix6;

Robot::Robot() {
  configs::TalonFXConfiguration configs{};
  configs.Slot0.kP = 30; // An error of 1 rotations results in 1.2 V output
  configs.Slot0.kI = 0; // No output for integrated error
  configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
  // Peak output of 8 V
  configs.Voltage.PeakForwardVoltage = 8_V;
  configs.Voltage.PeakReverseVoltage = -8_V;

  configs.Slot1.kP = 60; // An error of 1 rotations results in 60 A output
  configs.Slot1.kI = 0; // No output for integrated error
  configs.Slot1.kD = 6; // A velocity of 1 rps results in 6 A output
  // Peak output of 120 amps
  configs.TorqueCurrent.PeakForwardTorqueCurrent = 120_A;
  configs.TorqueCurrent.PeakReverseTorqueCurrent = -120_A;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_fx.GetConfigurator().Apply(configs);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status.GetName() << std::endl;
  }

  /* Make sure we start at 0 */
  m_fx.SetPosition(0_tr);


    configs::TalonFXConfiguration configs2{};

  /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
  configs2.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  configs2.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  configs2.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  configs2.Slot0.kI = 0; // No output for integrated error
  configs2.Slot0.kD = 0; // No output for error derivative
  // Peak output of 8 volts
  configs2.Voltage.PeakForwardVoltage = 8_V;
  configs2.Voltage.PeakReverseVoltage = -8_V;
  
  /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
  configs2.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
  configs2.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
  configs2.Slot1.kI = 0; // No output for integrated error
  configs2.Slot1.kD = 0; // No output for error derivative
  // Peak output of 40 A
  configs2.TorqueCurrent.PeakForwardTorqueCurrent = 40_A;
  configs2.TorqueCurrent.PeakReverseTorqueCurrent = -40_A;

  /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status2 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status2 = m_fllr.GetConfigurator().Apply(configs2);
    if (status2.IsOK()) break;
  }
  if (!status2.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status2.GetName() << std::endl;
  }

  // configs::TalonFXConfiguration cfg{};

  // /* Configure gear ratio */
  // configs::FeedbackConfigs &fdb = cfg.Feedback;
  // fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation

  // /* Configure Motion Magic */
  // configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  // mm.MotionMagicCruiseVelocity = 5_tps; // 5 (mechanism) rotations per second cruise
  // mm.MotionMagicAcceleration = 10_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // // Take approximately 0.1 seconds to reach max accel 
  // mm.MotionMagicJerk = 100_tr_per_s_cu;

  // configs::Slot0Configs &slot0 = cfg.Slot0;
  // slot0.kS = 0; // Add 0.25 V output to overcome static friction
  // slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  // slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  // slot0.kP = 0.5; // A position error of 0.2 rotations results in 12 V output
  // slot0.kI = 0; // No output for integrated error
  // slot0.kD = 0.02; // A velocity error of 1 rps results in 0.5 V output

  // ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  // for (int i = 0; i < 5; ++i) {
  //   status = m_fx.GetConfigurator().Apply(cfg);
  //   if (status.IsOK()) break;
  // }
  // if (!status.IsOK()) {
  //   std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  // }
  

  // m_fllr.SetControl(controls::Follower{m_fx.GetDeviceID(), true});
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

        frc::SmartDashboard::PutNumber("joystick_Y",m_joystick.GetLeftY());
        frc::SmartDashboard::PutNumber("joystick",m_joystick.GetLeftBumperButton());

  auto desiredRotations = m_joystick.GetLeftY() * 5_tr; // Go for plus/minus 10 rotations

  if (units::math::abs(desiredRotations) <= 0.001_tr) { // joystick deadzone
    desiredRotations = 0_tr;
  }

  if (m_joystick.GetLeftBumperButton()) {
    /* Use position voltage */
    m_fx.SetControl(m_positionVoltage.WithPosition(desiredRotations));
  } 
  // else if (m_joystick.GetRightBumperButton()) {
  //   /* Use position torque */
  //   m_fx.SetControl(m_positionTorque.WithPosition(desiredRotations));
  // } 
  else {
    /* Disable the motor instead */
    m_fx.SetControl(m_brake);
  }


  double joyValue = m_joystick.GetRightY();
  if (fabs(joyValue) < 0.1) joyValue = 0;

  auto desiredRotationsPerSecond = joyValue * 50_tps; // Go for plus/minus 50 rotations per second
        frc::SmartDashboard::PutNumber("desiredRotations",desiredRotationsPerSecond.value());

  if (m_joystick.GetRightBumperButton()) {
    /* Use velocity voltage */
    m_fllr.SetControl(m_velocityVoltage.WithVelocity(desiredRotationsPerSecond));
  }
  //  else if (m_joystick.GetRightBumperButton()) {
  //   /* Use velocity torque */
  //   m_fllr.SetControl(m_velocityTorque.WithVelocity(desiredRotationsPerSecond));
  // } 
  else {
    /* Disable the motor instead */
    m_fllr.SetControl(m_brake);
  }

  //   /* Deadband the joystick */
  // double leftY = m_joystick.GetLeftY();

  // if (fabs(leftY) < 0.1) leftY = 0;
  // auto desiredRotations = leftY * 5_tr; // Go for plus/minus 10 rotations
  //       frc::SmartDashboard::PutNumber("desiredRotations",desiredRotations.value());

  // if (m_joystick.GetLeftBumperButton()) {
  // m_fx.SetControl(m_mmReq.WithPosition(desiredRotations).WithSlot(0));
  // }
  // else {
  //   m_fx.SetControl(m_brake);
  // }

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
