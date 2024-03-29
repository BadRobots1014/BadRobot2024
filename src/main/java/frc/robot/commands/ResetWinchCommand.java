package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetWinchCommand extends Command {
  private final ShooterSubsystem m_subsystem;

  public ResetWinchCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.resetWinchEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
