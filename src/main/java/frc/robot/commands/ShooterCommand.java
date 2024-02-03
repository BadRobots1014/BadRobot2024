package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An shoot command that uses an shooter subsystem. */
public class ShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ShooterSubsystem m_subsystem;

    public static enum CommandType {
        StartMotor, Shoot
    };

    private final CommandType m_command;

    public ShooterCommand(ShooterSubsystem subsystem, CommandType command) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_command = command;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_command == CommandType.StartMotor) {
            if (!ShooterSubsystem.isShooterRunning()) {
                m_subsystem.runShooter();
            } else {
                m_subsystem.stopShooter();
            }
        } else if (m_command == CommandType.Shoot) {
            // Finish making shootCycle command in shooter subsystem
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
