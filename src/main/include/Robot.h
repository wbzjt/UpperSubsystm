// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
  // ctre::phoenix6::hardware::TalonFX m_fx{3, "rio"};

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  ctre::phoenix6::controls::PositionVoltage m_positionVoltage =
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
  /* Start at position 0, use slot 1 */
  ctre::phoenix6::controls::PositionTorqueCurrentFOC m_positionTorque =
      ctre::phoenix6::controls::PositionTorqueCurrentFOC{0_tr}.WithSlot(1);
  /* Keep a brake request so we can disable the motor */
  ctre::phoenix6::controls::StaticBrake m_brake{};

  ctre::phoenix6::CANBus kCANBus{"rio"};
  ctre::phoenix6::hardware::TalonFX m_fllr{13, kCANBus}; // Shoot
  ctre::phoenix6::hardware::TalonFX m_fx{12, kCANBus}; // Catch

    /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  ctre::phoenix6::controls::VelocityVoltage m_velocityVoltage =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

  /* Start at velocity 0, use slot 1 */
  // ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocityTorque =
  //     ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps}.WithSlot(1);
  /* Keep a neutral out so we can disable the motor */
  // ctre::phoenix6::controls::NeutralOut m_brake{};

  // initialize torque current FOC request with 0 amps
  ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);

  ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest2 = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);


// mutate request with output 

  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};

  frc::XboxController m_joystick{0};

 public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
