package frc.robot.commands;

import java.sql.Time;
import java.util.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final String m_commandType;
  private final boolean m_stop;
  private double startTime;
  private double lastTime;
  private double totalSpunUpTime;

  public ShooterCommand(ShooterSubsystem subsystem, String commandType, boolean stopOnSpunUp) {
    m_commandType = commandType;
    m_subsystem = subsystem;
    m_stop = stopOnSpunUp;
    addRequirements(subsystem);
  }

  public ShooterCommand(ShooterSubsystem subsystem) {
    m_commandType = "both";
    m_subsystem = subsystem;
    m_stop = false;
    addRequirements(subsystem);
  }

  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_commandType.equals("both")) m_subsystem.runShooter();
    else if (m_commandType.equals("front")) m_subsystem.runShooter(0.0);
    // else if (m_commandType.equals("index")) m_subsystem.runIndex();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_commandType.equals("both") || m_commandType.equals("front") || m_commandType.equals("all")) m_subsystem.stopShooter();
    // if (m_commandType.equals("index") || m_commandType.equals("all")) m_subsystem.stopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_stop) return false;
    if (m_stop && m_subsystem.getSpunUp() && totalSpunUpTime < 500) {
      totalSpunUpTime += System.currentTimeMillis() - lastTime;
      lastTime = System.currentTimeMillis();
      return false;
    }
    else if (m_stop && totalSpunUpTime >= 500) {
      lastTime = System.currentTimeMillis();
      if (m_subsystem.getSpunUp()) totalSpunUpTime = 0;
      return m_subsystem.getSpunUp();
    }
    else return false;
  }
}
