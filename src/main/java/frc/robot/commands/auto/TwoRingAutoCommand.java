// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.SetPoseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoRingAutoCommand extends SequentialCommandGroup {
  public TwoRingAutoCommand(ShooterSubsystem shoot, SwerveSubsystem swerve, IntakeSubsystem intake, Pose2d startingOffset, Supplier<Double> delay) {
    super(
      new SetPoseCommand(swerve, startingOffset).withTimeout(0),
      new WaitCommand(delay.get()),
      new ShootCommand(shoot, intake).withTimeout(4),
      new ParallelCommandGroup(
        new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(.3), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(-1)),
        new GroundIntakeCommand(intake)
      ).withTimeout(1.8),
      new ParallelCommandGroup(
        new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(-.3), supplyDouble(0), true, supplyBoolean(true), supplyBoolean(false), supplyDouble(-1)),
        new RetractIntakeCommand(intake)
      )
      .withTimeout(1.7),
      new ShootCommand(shoot, intake).withTimeout(4)
    );
  }

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
