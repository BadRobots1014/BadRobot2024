package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand (ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            new ParallelRaceGroup(
                new AirIntakeCommand(intakeSubsystem),
                new ShooterCommand(shooterSubsystem, "both", true)
            ),
            new ParallelCommandGroup(
                new ShooterCommand(shooterSubsystem, "both", false),
                new FeedShooterCommand(intakeSubsystem)
            )
        );
    }
}
