// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class GroundIntakeCommand extends Command {
  private final IntakeSubsystem m_subsystem;

  public GroundIntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.intakeFromGround();
    if(m_subsystem.m_intakeLimitSwitch.isPressed()){
      new RumbleCommand(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new StopRumbleCommand().execute();
    m_subsystem.stopEverything();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.m_intakeLimitSwitch.isPressed();
    // return false;
  }
}
