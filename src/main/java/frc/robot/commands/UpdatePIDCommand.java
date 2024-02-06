// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class UpdatePIDCommand extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final SwerveSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public UpdatePIDCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double p = m_subsystem.p.getDouble(ModuleConstants.kTurningP);
    double i = m_subsystem.i.getDouble(ModuleConstants.kTurningI);
    double d = m_subsystem.d.getDouble(ModuleConstants.kTurningD);
    m_subsystem.frontLeft.turningPidController.setPID(p, i, d);
    m_subsystem.frontRight.turningPidController.setPID(p, i, d);
    m_subsystem.backLeft.turningPidController.setPID(p, i, d);
    m_subsystem.backRight.turningPidController.setPID(p, i, d);
    System.out.println(p);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
