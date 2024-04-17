package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class HapticsCommand extends SequentialCommandGroup {
    public HapticsCommand (int controller, int direction) {
        addCommands(
            new RumbleCommand(controller, direction).withTimeout(2)
        );
    }
}
