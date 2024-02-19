package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem m_subsystem;
  private final String commandType;
  private boolean m_Finished = false;

  public ShooterCommand(ShooterSubsystem subsystem, String CommandType) {
    commandType = CommandType;
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
    if (commandType == "front") {
      m_subsystem.runShooter(0.0);
      System.out.println("Running front shooter motor...");
    }
    else if (commandType == "both") {
      m_subsystem.runShooter();
      System.out.println("Running both shooter motors...");
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
    return m_Finished;
  }
}
