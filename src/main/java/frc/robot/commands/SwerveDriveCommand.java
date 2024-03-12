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
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, rightJoystickYAxis, pov;
  public Supplier<Boolean> fastModeFunction, degreeSnap, fasterModeFunction;
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
    Supplier<Boolean> fasterMode,
    Supplier<Boolean> degSnap,
    Supplier<Double> povSupplier
  ) {
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    degreeSnap = degSnap;
    rightJoystickYAxis = rightJoystickY;
    fasterModeFunction = fasterMode;
    pov = povSupplier;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);

    m_tab = Shuffleboard.getTab("Swerve Instance");
    shuffleFieldOriented = m_tab.add("Field Oriented" + this.toString(), fieldOriented).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    m_tab.addNumber("Joystick Angle", this::getJoystickAngle);
    m_tab.addNumber("Target theta", this::getTargetTheta);
  }

  @Override
  public void execute() {
    // Get inputs
    
    //boolean fastMode = fastModeFunction.get();
    double rightJoystickAngle = getJoystickAngle();
    double currentHeading = swerveSubsystem.getHeading();
    double targetTheta = getTargetTheta(currentHeading);
    double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
    boolean fastMode = false, fasterMode = false;
    
    //Theta snap
    double deltaTheta = targetTheta - swerveSubsystem.getHeading();
    double thetaSpeed = deltaTheta * DriveConstants.kThetaMultiplier;

    //Hyjack right joystick for snapping

    if(degreeSnap.get()){
    System.out.println("SNIP SNIP YOUR SHINS ARE ABOUT TO BE  NIPPED");
    }
    if(degreeSnap.get() && Math.abs(xSpeed) < OIConstants.kDriveDeadband && Math.abs(ySpeed) < OIConstants.kDriveDeadband){
      turningSpeed = MathUtil.clamp(thetaSpeed,-1.0,1.0);
      System.out.println("SNIP SNIP YOUR SHINS HAVE BEEN NIPPED");
    }
    if(pov.get() != -1){
      xSpeed = pov.get() == 90 ? -DriveConstants.kNudgeSpeed : (pov.get() == 270 ? DriveConstants.kNudgeSpeed : 0);
      ySpeed = pov.get() == 0 ? DriveConstants.kNudgeSpeed : (pov.get() == 180 ? -DriveConstants.kNudgeSpeed : 0);
    }

    if(pov.get() == -1 && !degreeSnap.get()){
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
      fastMode = fastModeFunction.get();
      fasterMode = fasterModeFunction.get();
    }

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;
    thetaSpeed = Math.abs(thetaSpeed) > DriveConstants.kThetaDeadband ? thetaSpeed : 0;

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

  private double getJoystickAngle() {
    return Math.toDegrees(Math.atan(rightJoystickYAxis.get()/turningSpdFunction.get()));// / Math.PI / 2 * 360;
  }

  private double getTargetTheta(double heading) {
    while (heading < 0) heading += 360;
    while (heading > 359) heading -= 360;
    return (getJoystickAngle() - 90) % 360;
  }

  private double getTargetTheta() {
    return getTargetTheta(swerveSubsystem.getHeading());
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
