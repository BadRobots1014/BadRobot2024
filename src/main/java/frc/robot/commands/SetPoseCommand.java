package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SetPoseCommand extends Command {

  private final SwerveSubsystem m_subsystem;
  private final Pose2d m_pose;

  public SetPoseCommand(SwerveSubsystem subsystem, Pose2d pose) {
    m_subsystem = subsystem;
    m_pose = pose;
  }

  @Override
  public void execute() {
    m_subsystem.resetPose(m_pose);
  }
}
