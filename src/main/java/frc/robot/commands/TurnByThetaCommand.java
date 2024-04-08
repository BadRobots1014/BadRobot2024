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

public class TurnByThetaCommand extends Command {

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
  private double targetTheta, initialHeading;

  public TurnByThetaCommand(SwerveSubsystem subsystem, double turnDegrees) {
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    targetTheta = turnDegrees;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize(){
    swerveSubsystem.resetPose();
    isTurnFinished = false;
    initialHeading = swerveSubsystem.getHeading();
    if(initialHeading < 0){
      initialHeading = initialHeading + 180 + 360;
    }
  }

  @Override
  public void execute() {
    //autoturny stuffs
    targetTheta += initialHeading;
    double currentHeading = swerveSubsystem.getHeading();
    double currentRawHeading = swerveSubsystem.getHeading();
    double theta,speed;
    if(currentHeading < 0){
      currentHeading = currentHeading + 180 + 360;
    }
    currentHeading %= 360;
    theta = targetTheta - currentHeading;
    
    
    if(targetTheta < 0 && initialHeading >= 0 && initialHeading < 180 + targetTheta){
      if(currentHeading < 10){
        theta -= .1; //keep robot moving over the boundery
      }
      if(currentHeading < 0 || currentHeading > 180 + targetTheta){
        theta = (targetTheta + 360) - currentHeading; //robot move in correct direction once over boundery
      }
    }else if(targetTheta > 0 && initialHeading <= 0 && initialHeading > 180 + targetTheta){
      if(currentHeading > 350){
        theta += .1; //keep robot moving over the boundery
      }
      if(currentHeading > 359 || currentHeading < targetTheta){
        theta = (360 - targetTheta) - currentHeading;//robot move in correct direction once over boundery
      }
    }
    speed = theta / 45;
    System.out.println("DeltaTheta:" + theta);
    System.out.println("TargetTheta:" + targetTheta);
    System.out.println("Current Heading:" + currentHeading);
    

    

    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fastMode = fastModeFunction.get();

    //Hyjack joysticks
    turningSpeed = MathUtil.clamp(speed, -DriveConstants.kTurnThetaMaxSpeed, DriveConstants.kTurnThetaMaxSpeed);  //You's we;come constant police
    xSpeed = 0;
    ySpeed = 0;
    System.out.println("TurningSpeed:" + turningSpeed);

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    //Actual death
    if(Math.abs(turningSpeed) < DriveConstants.kTurnThetaShutoffSensitivity){ //TODO may need to adjust how sensitive it is
      isTurnFinished = true;
    }
    
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
