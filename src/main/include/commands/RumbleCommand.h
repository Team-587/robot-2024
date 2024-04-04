// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/WaitCommand.h>
#include <frc/XboxController.h>

#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RumbleCommand
    : public frc2::CommandHelper<frc2::Command, RumbleCommand> {
 public:
  RumbleCommand(
    frc::GenericHID::RumbleType RumbleType,
    units::second_t RumbleOnTime,
    units::second_t RumbleOffTime,
    double RumbleIntensity);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc::XboxController m_DriverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_CodriverController{OIConstants::kCoDriverControllerPort};

  frc2::WaitCommand m_WaitRumbleOnCommand;
  frc2::WaitCommand m_WaitRumbleOffCommand;

  frc::GenericHID::RumbleType m_RumbleType;
  double m_RumbleIntensity;
};
