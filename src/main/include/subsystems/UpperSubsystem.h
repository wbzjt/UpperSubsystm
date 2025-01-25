// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/XboxController.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "units/math.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/WaitCommand.h>
using namespace ctre::phoenix6;

class UpperSubsystem : public frc2::SubsystemBase {
 public:
  
  UpperSubsystem(frc::XboxController &_joystick) : m_joystick(_joystick) {
    FortMotorInit();
  }

  /**
   * Example command factory method.
   */
  frc2::CommandPtr UpperMethodCommand();

  /**
   * @brief 使用遥控器控制上层机构
   * 
   */
  frc2::CommandPtr TeleopControlCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool UpperCondition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  /** 炮台电机相关初始化 */
    void FortMotorInit();

    ctre::phoenix6::controls::PositionVoltage m_positionVoltage =
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0); // 位置环控制request
    ctre::phoenix6::controls::VelocityVoltage m_velocityVoltage =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0); // 速度环控制request
      // initialize torque current FOC request with 0 amps
    ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);

    ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest2 = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz);

    ctre::phoenix6::controls::StaticBrake m_brake{}; // 静态制动
    // initialize torque current FOC request with 0 amps
    controls::TorqueCurrentFOC m_motorRequest{0_A};

    ctre::phoenix6::CANBus kCANBus{"rio"}; // CAN总线
    ctre::phoenix6::hardware::TalonFX m_shoot{13, kCANBus}; // Shoot电机
    ctre::phoenix6::hardware::TalonFX m_catch{12, kCANBus}; // Catch电机

    uint8_t m_catchmode = 0; // 抓球电机控制模式：0-制动模式，1-位置环模式，2-电流环模式
    uint8_t m_shootmode = 0; // 射球电机控制模式： 0-制动模式，1-速度环模式，2-电流环模式

    double m_shootSpeed = 0.0; // Shoot电机目标速度
    double m_catchAngle = 0.0; // Catch电机目标角度
    double m_catchcurrent = 0.0; // Catch电机目标电流
    double m_shootcurrent = 0.0; // Shoot电机目标电流

    double m_catchgetAngle = 0.0; // Catch电机实际角度
    double m_catchoffset = 0.0; // Catch电机角度偏移(相对于机械零点)

    frc::XboxController& m_joystick;

    bool forwardlimit = 0;
    bool reverselimit = 0;
    void protectCatchAngle(); // 保护抓球电机角度

    bool Is_Reset = 0; // 是否复位(每次上电后需要复位，以确认电机相对于机构的绝对位置)
    void UpperReset(); // 上层机构复位

  public:
    frc2::CommandPtr DefaultCommand(); // 默认命令，使用手柄控制上层机构
    frc2::CommandPtr AutoCatchCommand(); // 自动抓球命令
    frc2::CommandPtr AutoPutCommand(); // 自动放置海藻
    frc2::CommandPtr AutoShootCommand(double _speed); // 自动射珊瑚
    // 炮台电机命令接口
    frc2::CommandPtr SetShootSpeedCommand(double speed);

    frc2::CommandPtr SetCatchAngleCommand(double angle);
    frc2::CommandPtr SetCatchCurrentCommand(double current);
    frc2::CommandPtr SetShootCurrentCommand(double current);

};
