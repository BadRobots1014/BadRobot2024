// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class TurnAmountAutoCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  //private final NavXGyroSubsystem m_gyroSubsystem;

  //heading is the direction the shin in need of breaking is
  //0 is forwards, 90 is right, 180 is back, 270 is left
  public TurnAmountAutoCommand(double heading, ShooterSubsystem shoot, SwerveSubsystem swerve) {
    
    
    super(
      new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(0), supplyDouble(0), supplyBoolean(false), supplyBoolean(false))
      //.onlyWhile()
    );
  }
  

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
