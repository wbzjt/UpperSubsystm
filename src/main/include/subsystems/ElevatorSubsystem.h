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
#include <frc2/command/button/CommandXboxController.h>

using namespace ctre::phoenix6;

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  
  ElevatorSubsystem(    frc2::CommandXboxController& m_joystick
) : m_joystick(m_joystick) {
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


    ctre::phoenix6::controls::StaticBrake m_brake{}; // 静态制动


    ctre::phoenix6::CANBus kCANBus{"rio"}; // CAN总线

  ctre::phoenix6::hardware::TalonFX m_stretch{17, kCANBus}; // 抓取电机
  ctre::phoenix6::hardware::TalonFX m_climb{18, kCANBus}; // 爬升电机

    ctre::phoenix6::controls::VelocityVoltage m_velocityVoltage =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0); // 速度闭环控制

    ctre::phoenix6::controls::TorqueCurrentFOC m_motorFOCRequest = ctre::phoenix6::controls::TorqueCurrentFOC{0_A}.WithUpdateFreqHz(500_Hz); // 电流控制

    double m_stretchSpeed = 0; // 抓取电机速度
    double m_climbcurrent = 0; // 爬升电机电流


  
  ctre::phoenix6::hardware::TalonFX m_elevator1{16, kCANBus}; // 电梯电机
  ctre::phoenix6::hardware::TalonFX m_elevator2{15, kCANBus}; // 跟随电机

  double m_elevatorposition = 0; // 归一化：0-1
  double m_elevatorpositionoffset = 0; // 电梯电机位置偏移
  double m_elevatorGetposition = 0; // 电梯电机当前位置

  ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC m_motorMMRequest = ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC{0_tr}.WithUpdateFreqHz(500_Hz).WithSlot(0);

  ctre::phoenix6::controls::Follower m_motorFollower = ctre::phoenix6::controls::Follower{m_elevator1.GetDeviceID(), true}.WithUpdateFreqHz(500_Hz);



    frc2::CommandXboxController& m_joystick;


    void protectelevatorAngle(); // 保护抓球电机角度

    bool Is_Reset = 0; // 是否复位(每次上电后需要复位，以确认电机相对于机构的绝对位置)
    void UpperReset(); // 上层机构复位

  public:
    frc2::CommandPtr DefaultCommand(); // 默认命令，使用手柄控制上层机构


};
