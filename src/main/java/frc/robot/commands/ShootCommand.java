package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (
        ShooterSubsystem shooterSubsystem
    ) {
        addCommands(
            new ShooterCommand(shooterSubsystem, "both").withTimeout(1.75),
            new ShooterCommand(shooterSubsystem, "all")
        );
    }
}
