// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToOneOneAutoCommand extends SequentialCommandGroup {
  public DriveToOneOneAutoCommand(SwerveSubsystem swerve, Pose2d startingOffset) {
    super(
      new DriveToPositionCommand(swerve, supplyDouble(0), supplyDouble(-.3), supplyDouble(0), supplyBoolean(false), supplyBoolean(false), supplyBoolean(true))
      .withTimeout(3)
    );
  }

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
