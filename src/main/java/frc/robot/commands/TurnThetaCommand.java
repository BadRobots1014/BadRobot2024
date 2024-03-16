package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class TurnThetaCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  public final Supplier<Boolean> fieldOrientedFunction, fastModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public final Supplier<Boolean> fieldOriented = ()-> false;
  public final Supplier<Boolean> fastMode = ()-> false;
  public final Supplier<Double> xSupplier = ()-> 0.0;
  public final Supplier<Double> ySupplier = ()-> 0.0;
  public final Supplier<Double> turnSupplier = ()-> 0.0;
  private boolean isTurnFinished = false;
  private double initial_heading;
  private double targetTheta;

  public TurnThetaCommand(SwerveSubsystem subsystem, double targetDegrees) {
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    targetTheta = targetDegrees;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize(){
    swerveSubsystem.resetPose();
    initial_heading = (swerveSubsystem.getHeading() + 360) % 360;
    isTurnFinished = false;
  }

  @Override
  public void execute() {
    //Fix heading bc gyro goes from 0 to 180 and then -180 back to 0
    double current_heading = (swerveSubsystem.getHeading() + 360) % 360;
    //double heading_difference = (targetTheta - current_heading) - 180;


    //autoturny stuffs
    double theta = targetTheta - current_heading;
    double speed = theta / 45;

    if(theta < 180){
      speed *= -1;
    }
    System.out.println("DeltaTheta:" + theta);
    System.out.println("Speed:" + speed);

    if(Math.abs(speed) < 0.005){ //TODO may need to adjust how sensitive it is
      isTurnFinished = true;
    }

    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fastMode = fastModeFunction.get();

    //Hyjack joysticks
    turningSpeed = MathUtil.clamp(speed, -1.0,1.0);
    xSpeed = 0;
    ySpeed = 0;
    System.out.println("TurningSpeed:" + turningSpeed);

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    // Slew soup
    double maxDriveSpeed = fastMode ? DriveConstants.kFastTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec;
    double maxTurnSpeed = fastMode ? DriveConstants.kFastTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec;
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * maxTurnSpeed;

    // I am speed
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // Field oriented
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // Divide and conker
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Actually do the thing
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return isTurnFinished;
  }
}
