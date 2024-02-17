// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {

    // TODO turn field oriented on or off
    public static final boolean kFieldOriented = true;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = .6;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.75);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.75);
    // Positions of modules relative to the center of mass
    public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // Front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2)
      ); // Back right

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontRightChassisAngularOffset =
      Math.PI -
      Math.PI / 2 -
      /* Additional correction because stupid module */2;
    public static final double kBackRightChassisAngularOffset =
      -Math.PI / 2 - Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = 0 - Math.PI / 2;
    public static final double kFrontLeftChassisAngularOffset =
      Math.PI / 2 - Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 21;
    public static final int kRearLeftDrivingCanId = 31;
    public static final int kFrontLeftDrivingCanId = 41;

    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 32;
    public static final int kFrontLeftTurningCanId = 42;

    public static final int kFrontRightEncoderCanId = 13;
    public static final int kRearRightEncoderCanId = 23;
    public static final int kRearLeftEncoderCanId = 33;
    public static final int kFrontLeftEncoderCanId = 43;

    public static final boolean kGyroReversed = false;

    // Reverse encoders if needed; note that this will break everything if you don't
    // go through and fix everything afterward
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontLeftAbsoluteEncoderReversed = true;

    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kFrontRightAbsoluteEncoderReversed = true;

    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftAbsoluteEncoderReversed = true;

    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    public static final boolean kBackRightAbsoluteEncoderReversed = true;

    public static final long kBootupDelay = 1000; // milliseconds of delay to allow the navx to start up

    public static final double kXSlewRateLimit = 2; // TODO adjust slew limits
    public static final double kYSlewRateLimit = 2;
    public static final double kTurnSlewRateLimit = 10;

    public static final double kTeleMaxRadiansPerSec = Math.PI / 2; // TODO adjust max teleop speeds
    public static final double kFastTeleMaxRadiansPerSec = Math.PI;
    public static final double kTeleMaxMetersPerSec = 0.6;
    public static final double kFastTeleMaxMetersPerSec = 0.9;

    public static final double kJoystickDeadzone = 0;

    public static final Button kTestMotorButton = Button.kLeftBumper;
  }

  public static final class ModuleConstants {

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kDriveMotorGearRatio = 8.14;
    public static final double kTurningMotorGearRatio = 12.8;
    public static final double kDriveEncoderRot2Meter =
      kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad =
      kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec =
      kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec =
      kTurningEncoderRot2Rad / 60;
    public static final double kModuleDeadband = 0.005;
    public static final double kTurningP = .9;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningPeriod = .005;
  }

  public static final class OIConstants {

    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  // TODO These are old and should be removed
  @Deprecated
  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
      );
  }

  public static final class ShooterConstants {

    /* Make sure to change these to whatever can IDs you guys want */
    public static final int kFrontMotorCanId = 61;
    public static final int kBackMotorCanId = 62;
  }
}
