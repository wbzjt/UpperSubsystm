// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"



frc2::CommandPtr ElevatorSubsystem::UpperMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool ElevatorSubsystem::UpperCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ElevatorSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  frc::SmartDashboard::PutNumber("Elevator1 Position", m_elevator1.GetPosition().GetValue().value());
  frc::SmartDashboard::PutNumber("Elevator1 Velocity", m_elevator1.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber("Elevator1 Current", m_elevator1.GetTorqueCurrent().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Elevator2 Position", m_elevator2.GetPosition().GetValue().value());
  frc::SmartDashboard::PutNumber("Elevator2 Velocity", m_elevator2.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber("Elevator2 Current", m_elevator2.GetTorqueCurrent().GetValueAsDouble());
  // frc::SmartDashboard::PutNumber("Elevator1 Voltage", m_elevator1.GetBusVoltage().GetValue().value());


  protectelevatorAngle(); // 保护抓球电机角度


  auto desirestretchspeed = m_stretchSpeed * 1_tps;
  m_stretch.SetControl(m_velocityVoltage.WithVelocity(desirestretchspeed));

  auto desireclimbcurrent = m_climbcurrent * 1_A;
  m_climb.SetControl(m_motorFOCRequest.WithOutput(desireclimbcurrent));

  auto desireelevatorposition = m_elevatorposition * -36_tr + m_elevatorpositionoffset * 1_tr;
  m_elevator1.SetControl(m_motorMMRequest.WithPosition(desireelevatorposition));

// -0.18
  
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void ElevatorSubsystem::FortMotorInit() {
  // 配置电机参数

    // elevator电机config配置
  configs::TalonFXConfiguration config1{};

  /* Configure Motion Magic */
  configs::MotionMagicConfigs &mm = config1.MotionMagic;
  mm.MotionMagicCruiseVelocity = 8.8 * 16.3636_tps; // 5 (mechanism) rotations per second cruise
  mm.MotionMagicAcceleration = 15 * 16.3636_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 150 * 16.3636_tr_per_s_cu;

  configs::Slot0Configs &slot0 = config1.Slot0;
  slot0.kG = -0.4; // Gear ratio of 1:2, 0.5 rotations per rotor rotation
  slot0.kS = 0.1; // Add 0.25 V output to overcome static friction
  slot0.kV = 0; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 0.8; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0; // No output for integrated error
  slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_elevator1.GetConfigurator().Apply(config1);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
  
  // Elevator2电机配置follow模式
  m_elevator2.SetControl(controls::Follower{m_elevator1.GetDeviceID(), false});


  // 抓取电机
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

    /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status2 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status2 = m_stretch.GetConfigurator().Apply(configs2);
    if (status2.IsOK()) break;
  }
  if (!status2.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status2.GetName() << std::endl;
  }

  m_stretch.SetControl(m_velocityVoltage.WithVelocity(0_tps));


  // 爬升电机
  configs::TalonFXConfiguration config3{};

      /* Retry config apply up to 5 times, report if failure */
  ctre::phoenix::StatusCode status3 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status3 = m_climb.GetConfigurator().Apply(config3);
    if (status3.IsOK()) break;
  }
  if (!status3.IsOK()) {
    std::cout << "Could not apply configs, error code: " << status3.GetName() << std::endl;
  }

  m_climb.SetControl(m_motorFOCRequest.WithOutput(0_A));




}

frc2::CommandPtr ElevatorSubsystem::TeleopControlCommand() {
  // Use the joystick to control the subsystem.
  return this->RunOnce(
      [this] {     
        // Use the joystick to control the subsystem.
        // if (Is_Reset){ // 复位完了可以手动控制


          if (m_joystick.LeftBumper().Get()) {

            if (abs(m_joystick.GetLeftY()) <= 0.05) { // joystick deadzone
              // desiredRotations = 0_tr;
            }
            else {
                  m_climbcurrent -= m_joystick.GetLeftY() * 0.5;
            }


          if (abs(m_joystick.GetRightY()) <= 0.05) { // joystick deadzone
              // desiredRotations = 0_tr;
            }
            else {
                  m_elevatorposition = m_joystick.GetRightY() * 0.5;
            }


        }

        if (m_joystick.RightBumper().Get()) {
          if (abs(m_joystick.GetRightY()) <= 0.05) { // joystick deadzone
              // desiredRotations = 0_tr;
            }
            else {
                  m_stretchSpeed = m_joystick.GetRightY() * 10;
            }
        }

        // else {}

      }
  );
}
 
void ElevatorSubsystem::protectelevatorAngle() {

  m_elevatorGetposition = (m_elevator1.GetPosition().GetValue() - m_elevatorpositionoffset * 1_tr) / -36_tr;
  // auto desireelevatorposition = m_elevatorposition * -36_tr + m_elevatorpositionoffset * 1_tr;


}

  frc2::CommandPtr ElevatorSubsystem::DefaultCommand() {

    return this->RunOnce(
      [this] {

        // /* 管理所有默认电机为制动 */
        // m_catchmode = 0;
        // m_shootmode = 0;


        // /* 手动控制上层机构 */
        // if (abs(m_joystick.GetLeftTriggerAxis()) > 0.05) {
        //   m_catchmode = 1;
        //   m_catchAngle = (1 - m_joystick.GetLeftTriggerAxis()) * 90;

        // } else {
        //   // m_catchmode = 0;
        // }

        // if (abs(m_joystick.GetRightTriggerAxis()) > 0.05) {
        //   m_shootmode = 1;
        //   m_shootSpeed = m_joystick.GetRightTriggerAxis() * 50;
        // } else {
        //   // m_shootmode = 0;
        // }
        // FortMotorInit();
      }
    );

  }

  void ElevatorSubsystem::UpperReset() {

    // static bool temp = 1;
    // if (temp) {
    //   m_catchAngle = m_catchgetAngle;
    //   temp = 0; // 将当前位置赋值给目标值
    // }
    
    // /* 复位夹爪Catch电机 */
    // m_catchmode = 1;
    // m_catchAngle -= 1;

    // if (m_catch.GetTorqueCurrent().GetValue().value() > 15) {
    //   m_catchoffset = m_catch.GetPosition().GetValueAsDouble() / 30 * 360 - 30;
    //   m_catchAngle = 0;
    //   Is_Reset = 1;
    // }
  }