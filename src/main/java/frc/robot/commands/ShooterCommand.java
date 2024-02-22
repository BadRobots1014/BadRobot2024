package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final String m_commandType;

  public ShooterCommand(ShooterSubsystem subsystem, String commandType) {
    m_commandType = commandType;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_commandType.equals("front")) {
      m_subsystem.runShooter(0.0);
    }
    else if (m_commandType.equals("both")) {
      m_subsystem.runShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopShooter();
    System.out.println("Stopping shooter...");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
