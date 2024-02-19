// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ShootAndDriveAutoCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootAndDriveAutoCommand(ShooterSubsystem shoot, SwerveSubsystem swerve) {
    super(
      // TODO: Make this drive distance and/or line up with the cone before the RunToPosition command should run

      new ShooterCommand(shoot).withTimeout(2),
      new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(.3), supplyDouble(0), supplyBoolean(false), supplyBoolean(false))
      .withTimeout(1)
    );
  }

  private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
  private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
}
