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
import frc.robot.commands.TurnThetaCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnAndShootAutoCommand extends SequentialCommandGroup {
  public TurnAndShootAutoCommand(ShooterSubsystem shoot, SwerveSubsystem swerve, Pose2d startingOffset, double turnDegrees, double delay) {
    super(
      new WaitCommand(delay),
      new SetPoseCommand(swerve, startingOffset).withTimeout(0),
      new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(-.3), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(-1))
      .withTimeout(1.9),
      new TurnThetaCommand(swerve, turnDegrees).withTimeout(1),
      new ShootCommand(shoot).withTimeout(4),
      new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(-.3), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(-1))
      .withTimeout(2.8)
    );
  }

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
