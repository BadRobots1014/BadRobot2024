package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 
    private final ShooterSubsystem m_subsystem;
    /*private enum ShooterStatus {MotorRunning, MotorStopped};
    private static ShooterStatus m_status;*/

    public static enum CommandType {StartMotor, Shoot};
    private final CommandType m_command;

    public ShooterCommand(ShooterSubsystem subsystem, CommandType command) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        m_command = command;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

     // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_command == CommandType.StartMotor) {
            // if start button, run
            if (!ShooterSubsystem.IsShooterRunning()) {
                m_subsystem.runShooter();
            }
            else {
                m_subsystem.stopShooter();
            }
            // if end button stop
        }
        else if (m_command == CommandType.Shoot) {
            // shoot thing
        }

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
