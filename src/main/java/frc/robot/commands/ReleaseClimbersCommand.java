// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ReleaseClimbersCommand extends Command {
  private final ClimberSubsystem m_subsystem;

  public ReleaseClimbersCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setClimberIdleMode(IdleMode.kCoast, IdleMode.kCoast);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setClimberIdleMode(IdleMode.kBrake, IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
