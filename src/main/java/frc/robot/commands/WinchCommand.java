package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class WinchCommand extends Command {

    private final ShooterSubsystem m_subsystem;
    private final Supplier<Double> m_power;

    public WinchCommand(ShooterSubsystem subsystem, Supplier<Double> power) {
        m_subsystem = subsystem;
        m_power = power;
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.manualWinch(m_power.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopWinch();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
