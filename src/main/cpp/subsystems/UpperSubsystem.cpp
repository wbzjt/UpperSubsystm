// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/UpperSubsystem.h"



frc2::CommandPtr UpperSubsystem::UpperMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([/* this */] { /* one-time action goes here */ });
}

bool UpperSubsystem::UpperCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void UpperSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  protectCatchAngle(); // 保护抓球电机角度

  double m_catchgetcurrent = m_catch.GetTorqueCurrent().GetValue().value();
  double m_shootgetcurrent = m_shoot.GetTorqueCurrent().GetValue().value();
      /** 在这个循环不断下发控制指令 */
    frc::SmartDashboard::PutNumber("CatchAngle", m_catchAngle);
    frc::SmartDashboard::PutNumber("ShootSpeed", m_shootSpeed);
    frc::SmartDashboard::PutNumber("m_catchgetAngle1", m_catch.GetPosition().GetValueAsDouble() / 30 * 360);
    frc::SmartDashboard::PutNumber("m_catchgetAngle", m_catchgetAngle);

    // frc::SmartDashboard::PutNumber("ShootSpeed", m_shootSpeed);
    frc::SmartDashboard::PutNumber("m_catchgetcurrent", m_catchgetcurrent);
    frc::SmartDashboard::PutNumber("m_shootgetcurrent", m_shootgetcurrent);
    frc::SmartDashboard::PutNumber("CatchMode", m_catchmode);
    frc::SmartDashboard::PutNumber("ShootMode", m_shootmode);
    frc::SmartDashboard::PutNumber("m_catchcurrent", m_catchcurrent);
    frc::SmartDashboard::PutNumber("m_shootcurrent", m_shootcurrent);

    frc::SmartDashboard::PutNumber("CatchCurrent_get", m_catch.GetTorqueCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Is_Reset", Is_Reset);
    frc::SmartDashboard::PutNumber("m_catchoffset", m_catchoffset);
    // frc::SmartDashboard::PutNumber("m_catchgetAngle", m_catchgetAngle);

  if (m_joystick.GetXButton() && !Is_Reset) {
    UpperReset(); // 对上层机构进行复位
  }
  else{}

    /* 抓球电机 */
    if (m_catchmode == 0) {
      /* Disable the motor instead */
      m_catch.SetControl(m_brake);
    } else if(m_catchmode == 1) {
      auto desiredRotations = (-m_catchAngle + m_catchoffset) / 360 * UpperConstants::CatchRatio * 1_tr; // 电机实际转动圈数(turns):catchAngle减少，夹爪张开

      if (fabs(desiredRotations.value()) <= 0.01) { //deadzone
        desiredRotations = 0_tr;
      }
      else {}


        m_catch.SetControl(m_positionVoltage.WithPosition(desiredRotations)); // 机械输出旋转圈数

    } else if (m_catchmode == 2)
    {

      auto catchdesiredCurrent = m_catchcurrent * 1_A; // 电机电流(A)
      if (units::math::abs(catchdesiredCurrent) <= 0.01_A) { // joystick deadzone
        catchdesiredCurrent = 0_A;
      }
      else {}
      m_catch.SetControl(m_motorFOCRequest.WithOutput(-catchdesiredCurrent).WithMaxAbsDutyCycle(0.4).WithLimitReverseMotion(reverselimit).WithLimitForwardMotion(forwardlimit));// -15A e

    }
    else {}
    
    /* 射球电机 */

    if (m_shootmode == 0) {
      m_shoot.SetControl(m_brake);
    } else if (m_shootmode == 1) {

      auto desiredRotationsPerSecond = m_shootSpeed * UpperConstants::ShootRatio * 1_tps; // 机械装置实际转动圈数每秒(turns/s)

      frc::SmartDashboard::PutNumber("desiredRotationsPerSecond", desiredRotationsPerSecond.value());
        /* Use velocity voltage */
        m_shoot.SetControl(m_velocityVoltage.WithVelocity(desiredRotationsPerSecond));// 机械装置实际每秒转动圈数(turns/s)


    } else if(m_shootmode == 2) {


        auto shootdesiredCurrent = m_shootcurrent * 1_A; // 机械装置实际转动圈数(turns)
        if (units::math::abs(shootdesiredCurrent) <= 0.01_A) { // joystick deadzone
          shootdesiredCurrent = 0_A;
        }
        else {}

        m_shoot.SetControl(m_motorFOCRequest2.WithOutput(-shootdesiredCurrent).WithMaxAbsDutyCycle(0.4));// 15A 

    }
  // }

  
}

void UpperSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

void UpperSubsystem::FortMotorInit() {
  // 配置电机参数

    // Catch电机config配置
    configs::TalonFXConfiguration configs1{};

    configs::FeedbackConfigs &configs1fdb = configs1.Feedback;
    // configs1fdb.SensorToMechanismRatio = UpperConstants::CatchRatio; // 30 rotor rotations per mechanism rotation

    configs1.Slot0.kS = 0.27; // To account for friction, add 0.1 V of static feedforward
    configs1.Slot0.kP = 4; // An error of 1 rotations results in 1.2 V output
    configs1.Slot0.kI = 0.001; // No output for integrated error
    configs1.Slot0.kD = 0.2; // A velocity of 1 rps results in 0.1 V output

    // Peak output of 8 V
    configs1.Voltage.PeakForwardVoltage = 8_V;
    configs1.Voltage.PeakReverseVoltage = -8_V;
    // Peak output of 120 amps
    configs1.TorqueCurrent.PeakForwardTorqueCurrent = 120_A;
    configs1.TorqueCurrent.PeakReverseTorqueCurrent = -120_A;

    /* Retry config apply up to 5 times, report if failure */
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = m_catch.GetConfigurator().Apply(configs1);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not apply configs, error code: " << status.GetName() << std::endl;
    }

    /* 确保catch电机位置为0 */
    m_catchAngle = 0;

    // Shoot电机config配置
    configs::TalonFXConfiguration configs2{};

    configs::FeedbackConfigs &configs2fdb = configs2.Feedback;
    // configs2fdb.SensorToMechanismRatio = UpperConstants::ShootRatio; // 1 rotor rotations per mechanism rotation

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs2.Slot0.kS = 0.4; // To account for friction, add 0.1 V of static feedforward
    configs2.Slot0.kV = 0.16; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs2.Slot0.kP = 0.3; // An error of 1 rotation per second results in 0.11 V output
    configs2.Slot0.kI = 0; // No output for integrated error
    configs2.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs2.Voltage.PeakForwardVoltage = 8_V;
    configs2.Voltage.PeakReverseVoltage = -8_V;
        // Peak output of 120 amps
    configs2.TorqueCurrent.PeakForwardTorqueCurrent = 120_A;
    configs2.TorqueCurrent.PeakReverseTorqueCurrent = -120_A;
    
    /* Retry config apply up to 5 times, report if failure */
    ctre::phoenix::StatusCode status2 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status2 = m_shoot.GetConfigurator().Apply(configs2);
        if (status2.IsOK()) break;
    }
    if (!status2.IsOK()) {
        std::cout << "Could not apply configs, error code: " << status2.GetName() << std::endl;
    }

    /* 确保shoot电机速度为0 */
    m_shootSpeed = 0;

}

frc2::CommandPtr UpperSubsystem::TeleopControlCommand() {
  // Use the joystick to control the subsystem.
  return this->RunOnce(
      [this] {     
        // Use the joystick to control the subsystem.
        // if (Is_Reset){ // 复位完了可以手动控制


          if (m_joystick.GetLeftBumperButton()) {

          // m_catchmode = 1;
          // m_catchAngle = -m_joystick.GetLeftY() * 90; // 允许遥控器控制夹爪转动范围为0-90度

            m_catchmode = 2;
            frc::SmartDashboard::PutNumber("m_joystick.GetLeftY()", m_joystick.GetLeftY());
            if (units::math::abs(m_joystick.GetLeftY() * 0.5_A) <= 0.03_A) { // joystick deadzone
              // desiredRotations = 0_tr;
            }
            else {
                  m_catchcurrent -= m_joystick.GetLeftY() * 0.5;
            }

          } else {}

          if (m_joystick.GetRightBumperButton()) {

          m_shootmode = 1;
          if (std::abs(m_joystick.GetRightY()) > 0.05) {
            m_shootSpeed = m_joystick.GetRightY() * 50;
          } else {
            m_shootSpeed = 0;
          }
          } else {}


        }

        // else {}

      // }
  );
}

void UpperSubsystem::protectCatchAngle() {
  // 保护抓球电机角度,设置两端角度限位，防止力矩控制时长时间顶住机械两端
  // m_catchgetAngle = m_catch.GetPosition().GetValue().value();
  frc::SmartDashboard::PutNumber("reverselimit", reverselimit);
  frc::SmartDashboard::PutNumber("forwardlimit", forwardlimit);

      auto desiredRotations = (-m_catchAngle + m_catchoffset) / 360 * UpperConstants::CatchRatio * 1_tr; // 电机实际转动圈数(turns)


  m_catchgetAngle = - ( (m_catch.GetPosition().GetValueAsDouble() / 30  * 360 ) - m_catchoffset) ;

  if (m_catchgetAngle > 100) {
    reverselimit = 1;
  }
  else if (m_catchgetAngle < 20) {
    forwardlimit = 1;
  }
  else {
    forwardlimit = 0;
    reverselimit = 0;
  }

}

  frc2::CommandPtr UpperSubsystem::DefaultCommand() {

    return this->RunOnce(
      [this] {

        // /* 管理所有默认电机为制动 */
        // m_catchmode = 0;
        // m_shootmode = 0;


        /* 手动控制上层机构 */
        if (abs(m_joystick.GetLeftTriggerAxis()) > 0.05) {
          m_catchmode = 1;
          m_catchAngle = (1 - m_joystick.GetLeftTriggerAxis()) * 90;

        } else {
          // m_catchmode = 0;
        }

        if (abs(m_joystick.GetRightTriggerAxis()) > 0.05) {
          m_shootmode = 1;
          m_shootSpeed = m_joystick.GetRightTriggerAxis() * 50;
        } else {
          // m_shootmode = 0;
        }
        // FortMotorInit();
      }
    );

  }

    frc2::CommandPtr UpperSubsystem::AutoCatchCommand() {
      return frc2::cmd::Sequence(

        this->SetCatchCurrentCommand(15),
        this->SetShootSpeedCommand(5),
        frc2::WaitUntilCommand([this] { return m_shoot.GetTorqueCurrent().GetValue().value() > 110; }).ToPtr(), // 当Shoot电机将球吸入底部时，此时电机会堵转，由此判断是否吸入
        this->SetShootCurrentCommand(-15)

      );
      
    }

    frc2::CommandPtr UpperSubsystem::AutoPutCommand() {
      return frc2::cmd::Sequence(

        this->SetShootSpeedCommand(-5),
        frc2::WaitUntilCommand([this] { return m_catchgetAngle > 110; }).ToPtr(), // 当夹爪张开
        this->SetCatchAngleCommand(10), // 根据不同高度传入不同速度
        this->SetShootSpeedCommand(0) // 根据不同高度传入不同速度

        
      );
    }

    frc2::CommandPtr UpperSubsystem::AutoShootCommand(double _speed) {
      return frc2::cmd::Sequence(

        this->SetCatchAngleCommand(10),
        frc2::WaitUntilCommand([this] { return m_catchgetAngle < 20; }).ToPtr(), // 当夹爪张开
        this->SetShootSpeedCommand(_speed), // 根据不同高度传入不同速度
        frc2::WaitCommand(1_s).ToPtr(),
        this->SetShootSpeedCommand(0) // 根据不同高度传入不同速度


      );
    }

    frc2::CommandPtr UpperSubsystem::SetShootSpeedCommand(double speed) {
      return this->RunOnce(
        [this, speed] {
          m_shootmode = 1;
          m_shootSpeed = speed;
        }
      );
    }

    frc2::CommandPtr UpperSubsystem::SetCatchAngleCommand(double angle) {
      return this->RunOnce(
        [this, angle] {
          m_catchmode = 1;
          m_catchAngle = angle;
        }
      );
    }

    frc2::CommandPtr UpperSubsystem::SetCatchCurrentCommand(double current) {
      return this->RunOnce(
        [this, current] {
          m_catchmode = 2;
          m_catchcurrent = current;
        }
      );
    }

    frc2::CommandPtr UpperSubsystem::SetShootCurrentCommand(double current) {
      return this->RunOnce(
        [this, current] {
          m_shootmode = 2;
          m_shootcurrent = current;
        }
      );
    }

  // frc2::CommandPtr UpperSubsystem::AutoControlCommand() {
  //   // Use the joystick to control the subsystem.
  //   return this->RunOnce(
  //       [this] {     
  //         // Use the joystick to control the subsystem.
  //         m_catchAngle = 90; // 允许遥控器控制夹爪转动范围为0-90度
  //         m_shootSpeed = 50; // 允许遥控器控制射球速度范围为0-50转每秒
  //       }
  //   );
  // }
  void UpperSubsystem::UpperReset() {

    static bool temp = 1;
    if (temp) {
      m_catchAngle = m_catchgetAngle;
      temp = 0; // 将当前位置赋值给目标值
    }
    
    /* 复位夹爪Catch电机 */
    m_catchmode = 1;
    m_catchAngle -= 1;

    if (m_catch.GetTorqueCurrent().GetValue().value() > 15) {
      m_catchoffset = m_catch.GetPosition().GetValueAsDouble() / 30 * 360 - 30;
      m_catchAngle = 0;
      Is_Reset = 1;
    }
  }