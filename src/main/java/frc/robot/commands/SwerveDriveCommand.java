package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveDriveCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, rightJoystickYAxis;
  public Supplier<Boolean> fastModeFunction, degreeSnap;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public boolean fieldOrientedFunction;
  private ShuffleboardTab m_tab;
  private GenericEntry shuffleFieldOriented;

  public SwerveDriveCommand(
    SwerveSubsystem subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    Supplier<Double> rightJoystickY,
    boolean fieldOriented,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> degSnap
  ) {
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    degreeSnap = degSnap;
    rightJoystickYAxis = rightJoystickY;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);

    m_tab = Shuffleboard.getTab("Field Oriented");
    shuffleFieldOriented = m_tab.add("Field Oriented", fieldOriented).getEntry();
  }

  @Override
  public void execute() {
    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fastMode = fastModeFunction.get();
    double rightYAxis = rightJoystickYAxis.get();
    double rightJoystickAngle = Math.atan(rightYAxis/turningSpeed);
    System.out.println("JOYSTICKANGLE: " + rightJoystickAngle);
    double currentHeading = swerveSubsystem.getHeading();
    if(currentHeading < 0){
      currentHeading += 360;
    }
    System.out.println("CurrentHeading: " + currentHeading);

    double targetTheta = rightJoystickAngle;

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed =
      Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    // Slew soup
    double maxDriveSpeed = fastMode ? DriveConstants.kFastTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec;
    double maxTurnSpeed = fastMode ? DriveConstants.kFastTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec;
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    turningSpeed =
      turningLimiter.calculate(turningSpeed) *
      maxTurnSpeed;


    double deltaTheta = targetTheta - swerveSubsystem.getHeading();

    if(Math.abs(deltaTheta/45) < 0.005){
        deltaTheta = 0; //to stop oscillations
    }

    //Hyjack right joystick for snapping
    if(degreeSnap.get() == true){
    turningSpeed = MathUtil.clamp((deltaTheta / 45),-1.0,1.0);
    }


    // I am speed
    ChassisSpeeds chassisSpeeds;
    if (shuffleFieldOriented.getBoolean(fieldOrientedFunction)) {
      // Field oriented
      chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          turningSpeed,
          swerveSubsystem.getRotation2d()
        );
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // Divide and conker
    SwerveModuleState[] moduleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Actually do the thing
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
