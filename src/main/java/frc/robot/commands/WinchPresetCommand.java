package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class WinchPresetCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Supplier<Double> m_goal;

  public WinchPresetCommand(ShooterSubsystem subsystem, Supplier<Double> goalPosition) {
    m_subsystem = subsystem;
    m_goal = goalPosition;
    addRequirements(subsystem);
  }

  public WinchPresetCommand(ShooterSubsystem subsystem, double goalPosition) {
    m_subsystem = subsystem;
    m_goal = () -> goalPosition;
    addRequirements(subsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_subsystem.getWinchEncoder() < m_goal.get()) {
    //   m_subsystem.winchDown();
    // }
    // else if (m_subsystem.getWinchEncoder() > m_goal.get()) {
    //   m_subsystem.winchUp();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_subsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//Math.abs(m_subsystem.getWinchEncoder() - m_goal.get()) < ShooterConstants.kWinchDeadBand;
  }
}
