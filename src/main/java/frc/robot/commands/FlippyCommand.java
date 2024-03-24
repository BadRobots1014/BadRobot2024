// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class FlippyCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private Supplier<Double> triggerVal;

  public FlippyCommand(ShooterSubsystem subsystem, Supplier<Double> Rtrig) {
    m_subsystem = subsystem;
    triggerVal = Rtrig;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {//should run both motors but run the flippy motor at the power of the trigger
    m_subsystem.runFlippyIntake(ShooterConstants.kFrontIntakePower);//can change later
    m_subsystem.runFlippyMotor(triggerVal.get());//will need to change to runtoposition once we figure out the encoder counts
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopFlippyIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
