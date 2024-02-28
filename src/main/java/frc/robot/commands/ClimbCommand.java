// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends Command {
  private final ClimberSubsystem m_subsystem;
  private final Supplier<Double> m_power;

  public ClimbCommand(ClimberSubsystem subsystem, double power) {
    m_subsystem = subsystem;
    m_power = () -> power;
    addRequirements(subsystem);
  }

  public ClimbCommand(ClimberSubsystem subsystem, Supplier<Double> power) {
    m_subsystem = subsystem;
    m_power = power;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runClimbers(m_power.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopClimbers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
