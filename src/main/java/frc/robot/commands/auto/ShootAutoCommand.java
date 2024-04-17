// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetPoseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootAutoCommand extends SequentialCommandGroup {
  public ShootAutoCommand(ShooterSubsystem shoot, SwerveSubsystem swerve, IntakeSubsystem intake, Pose2d startingOffset, Supplier<Double> delay) {
    super(
      new WaitCommand(delay.get()),
      new ShootCommand(shoot, intake).withTimeout(4),
      new SetPoseCommand(swerve, startingOffset).withTimeout(0),
      new ZeroHeadingCommand(swerve, startingOffset.getRotation().getDegrees() + 180).withTimeout(0)
    );
  }
}
