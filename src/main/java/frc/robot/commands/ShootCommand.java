package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new AirIntakeCommand(intakeSubsystem).withTimeout(0.2),
                new ShooterCommand(shooterSubsystem, "both")
            ).withTimeout(1.75),
            new ParallelCommandGroup(
                new ShooterCommand(shooterSubsystem, "both"),
                new FeedShooterCommand(intakeSubsystem)
            )
        );
    }
}
