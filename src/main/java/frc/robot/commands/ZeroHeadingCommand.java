package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeadingCommand extends Command {

  private final SwerveSubsystem m_subsystem;
  private static double offset;

  public ZeroHeadingCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem;
    offset = 0;
  }

  public ZeroHeadingCommand(SwerveSubsystem subsystem, double degreeOffset) {
    m_subsystem = subsystem;
    offset = degreeOffset;
  }

  @Override
  public void execute() {
    m_subsystem.resetPose(new Pose2d(0,0, Rotation2d.fromDegrees(offset)));
  }
}
