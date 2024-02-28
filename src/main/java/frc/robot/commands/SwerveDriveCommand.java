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

public class SwerveDriveCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  public Supplier<Boolean> fieldOrientedFunction, fastModeFunction, degreeSnap;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private boolean isFirstJoystickMove = false; //so it doesent go turning around every loop
  private boolean prevFirstJoystickState = false;
  private boolean isFirstJoystickActive = false;

  public SwerveDriveCommand(
    SwerveSubsystem subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    Supplier<Boolean> fieldOriented,
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
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0; //aka deadbands

    boolean fastMode = fastModeFunction.get();


    boolean degSnapMode = degreeSnap.get();
    double currentHeading = swerveSubsystem.getHeading();
    if(currentHeading < 0){
      currentHeading += 360;
    }
    System.out.println("CurrentHeading: " + currentHeading);
    double snappedLowestHeading = ( (int)(currentHeading/90) ) * 90; //snaps to the leftmost straight heading 
    System.out.println("snappedLowestHeading: " + snappedLowestHeading);
    double targetTheta = 0;

    if((Math.abs(turningSpeed) < 0.1) && degreeSnap.get()){//if joystick  is within 0.1 of 0
        //turningSpeed = 0;
        isFirstJoystickMove = false;
    }else{
        isFirstJoystickMove = true;
    }//so the targetTheta is only chosen once

    if(prevFirstJoystickState != isFirstJoystickMove){
        if((turningSpeed > 0)){//turning right
        isFirstJoystickMove = false;
        targetTheta = snappedLowestHeading % 360;
    }else if((turningSpeed < 0)){//turning left
        isFirstJoystickMove = false;
        targetTheta = (snappedLowestHeading + 90) % 360;
    }
        prevFirstJoystickState = isFirstJoystickMove;
    }

    System.out.println("isFirstJoystickMove" + isFirstJoystickMove);
    System.out.println("Target Theta" + targetTheta);
    
    double deltaTheta = targetTheta - swerveSubsystem.getHeading();

    if(Math.abs(deltaTheta/45) < 0.005){
        deltaTheta = 0; //to stop oscillations
    }

    //Hyjack right joystick for snapping
    if(degreeSnap.get() == true){
    turningSpeed = MathUtil.clamp((deltaTheta / 45),-1.0,1.0);
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

  public void toggleFieldOriented() {
    fieldOrientedFunction = new Supplier<Boolean>(){
      @Override
      public Boolean get() {
        return !fieldOrientedFunction.get();
      }
    };
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
