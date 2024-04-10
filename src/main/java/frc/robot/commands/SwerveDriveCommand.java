package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveDriveCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, pov, auxLTrig, auxRTrig;
  public Supplier<Boolean> fastModeFunction, fasterModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public boolean fieldOrientedFunction, isPresetTurnActive;
  private ShuffleboardTab m_tab;
  private GenericEntry shuffleFieldOriented;

  public SwerveDriveCommand(
    SwerveSubsystem subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    boolean fieldOriented,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> fasterMode,
    Supplier<Double> povSupplier,
    Supplier<Double> auxLeftTrigger,
    Supplier<Double> auxRightTrigger
  ) {
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    fasterModeFunction = fasterMode;
    pov = povSupplier;
    auxLTrig = auxLeftTrigger;
    auxRTrig = auxRightTrigger;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);

    m_tab = Shuffleboard.getTab("Swerve Instance");
    shuffleFieldOriented = m_tab.add("Field Oriented" + this.toString(), fieldOriented).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  }

  @Override
  public void execute() {
    // Get inputs
    double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
    boolean fastMode = false, fasterMode = false;
    if (pov.get() == -1) {
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
      fastMode = fastModeFunction.get();
      fasterMode = fasterModeFunction.get();
    }
    else {
      xSpeed = pov.get() == 90 ? -DriveConstants.kNudgeSpeed : (pov.get() == 270 ? DriveConstants.kNudgeSpeed : 0);
      ySpeed = pov.get() == 0 ? DriveConstants.kNudgeSpeed : (pov.get() == 180 ? -DriveConstants.kNudgeSpeed : 0);
    }
    double presetTurnSpeedFactor = 0;
    double targetTheta = 0;//set preset angle target
    isPresetTurnActive = (Math.abs(auxLTrig.get()) > OIConstants.kDriveDeadband || Math.abs(auxRTrig.get()) > OIConstants.kDriveDeadband);

    if(Math.abs(auxLTrig.get()) > OIConstants.kDriveDeadband){ //yes I know this is redundent
      targetTheta = DriveConstants.kSpeakerTheta;
      presetTurnSpeedFactor = auxLTrig.get();
    }else if(Math.abs(auxRTrig.get()) > OIConstants.kDriveDeadband){//Left will take precident over right
      targetTheta = DriveConstants.kSourceTheta;
      presetTurnSpeedFactor = auxRTrig.get();
    }

    presetTurnSpeedFactor = MathUtil.clamp(presetTurnSpeedFactor, -1.0, 1.0);

    //autoturny stuffs
    double currentHeading = swerveSubsystem.getHeading();
    if(currentHeading < 0){
      currentHeading = currentHeading + 180 + 360;
    }
    currentHeading %= 360;
    double currentRawHeading = swerveSubsystem.getHeading();
    double theta,speed;
    
    
    theta = targetTheta - currentHeading;
    if(targetTheta < 0 && currentHeading >= 0 && currentHeading < 180 + targetTheta){
      if(currentHeading < 10){
        theta -= .1; //keep robot moving over the boundery
      }
      if(currentHeading < 0 || currentHeading > 180 + targetTheta){
        theta = (targetTheta + 360) - currentHeading; //robot move in correct direction once over boundery
      }
    }else if(targetTheta > 0 && currentHeading <= 0 && currentHeading > 180 + targetTheta){
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
    System.out.println("Current Raw Heading:" + currentRawHeading);

    if(isPresetTurnActive){
      //if a trigger is held down override turningspeed
      turningSpeed = MathUtil.clamp(speed * presetTurnSpeedFactor, -DriveConstants.kTurnThetaMaxSpeed, DriveConstants.kTurnThetaMaxSpeed);
    }


    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    // Slew soup
    double maxDriveSpeed = fasterMode ? DriveConstants.kFasterTeleMaxMetersPerSec : (fastMode ? DriveConstants.kFastTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec);
    double maxTurnSpeed = fasterMode ? DriveConstants.kFasterTeleMaxRadiansPerSec : (fastMode ? DriveConstants.kFastTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec);
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * maxTurnSpeed;

    // I am speed
    ChassisSpeeds chassisSpeeds;
    if (shuffleFieldOriented.getBoolean(fieldOrientedFunction) && pov.get() == -1) {
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
