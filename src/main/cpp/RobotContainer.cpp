// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() : m_upperSubsystem(m_joystick)  {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_upperSubsystem.SetDefaultCommand(m_upperSubsystem.TeleopControlCommand());

  frc2::Trigger([this] {
    return m_joystick.GetYButton();
  }).OnTrue(m_upperSubsystem.AutoCatchCommand());  

  frc2::Trigger([this] {return m_joystick.GetAButton();}).OnTrue(
    m_upperSubsystem.AutoPutCommand()
  );

    frc2::Trigger([this] {return m_joystick.GetBButton();}).OnTrue(
    m_upperSubsystem.AutoShootCommand(-5)
  );

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
