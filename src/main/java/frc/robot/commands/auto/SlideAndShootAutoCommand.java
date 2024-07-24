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
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SlideAndShootAutoCommand extends SequentialCommandGroup {
  public SlideAndShootAutoCommand(ShooterSubsystem shoot, SwerveSubsystem swerve, IntakeSubsystem intake, Pose2d startingOffset, int direction) {
    super(
        new WaitCommand(4),
      new SetPoseCommand(swerve, startingOffset).withTimeout(0),
      new SwerveDriveCommand(swerve, supplyDouble(direction == 0 ? -.3 : .3), supplyDouble(0), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(0), supplyDouble(0), supplyDouble(0))
      .withTimeout(1),
      new ShootCommand(shoot, intake),
      new SwerveDriveCommand(swerve, supplyDouble(direction == 0 ? .3 : -.3), supplyDouble(0), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(0), supplyDouble(0), supplyDouble(0))
      .withTimeout(1),
      new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(-.3), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(0), supplyDouble(0), supplyDouble(0)),
      new ZeroHeadingCommand(swerve, startingOffset.getRotation().getDegrees() + 180).withTimeout(0)
    );
  }

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
