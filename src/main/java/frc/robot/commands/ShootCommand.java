package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (
        ShooterSubsystem shooterSubsystem
    ) {
        addCommands(
            new ShooterCommand(shooterSubsystem, "front").withTimeout(2.5),
            new ShooterCommand(shooterSubsystem, "both")
        );
    }
}
