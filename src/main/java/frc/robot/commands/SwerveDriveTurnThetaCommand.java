package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;
import java.util.function.Supplier;

public class SwerveDriveTurnThetaCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final NavXGyroSubsystem m_gyroSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  public final Supplier<Boolean> fieldOrientedFunction, fastModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public final Supplier<Boolean> fieldOriented = ()-> false;
  public final Supplier<Boolean> fastMode = ()-> false;
  public final Supplier<Double> xSupplier = ()-> 0.0;
  public final Supplier<Double> ySupplier = ()-> 0.0;
  public final Supplier<Double> turnSupplier = ()-> 0.0;
  private double initial_yaw;
  private double targetTheta;

  public SwerveDriveTurnThetaCommand(
    SwerveSubsystem subsystem,
    NavXGyroSubsystem gyro,// oh boy i love greek gyros
    double turnAmount
  ) {
    swerveSubsystem = subsystem;
    m_gyroSubsystem = gyro;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    targetTheta = turnAmount;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize(){
    m_gyroSubsystem.reset();
    initial_yaw = m_gyroSubsystem.getYaw();
  }

  @Override
  public void execute() {
    //autoturny stuffs
    double theta = initial_yaw - m_gyroSubsystem.getYaw();
    double speed = theta / 45;
    


    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fastMode = fastModeFunction.get();

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed =
      Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    //Hyjack joysticks

turningSpeed = MathUtil.clamp(speed, -1.0,1.0);


    // Slew soup
    double maxDriveSpeed = fastMode ? DriveConstants.kFastTeleMaxMetersPerSec : DriveConstants.kTeleMaxMetersPerSec;
    double maxTurnSpeed = fastMode ? DriveConstants.kFastTeleMaxRadiansPerSec : DriveConstants.kTeleMaxRadiansPerSec;
    xSpeed = xLimiter.calculate(xSpeed) * maxDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxDriveSpeed;
    turningSpeed =
      turningLimiter.calculate(turningSpeed) *
      maxTurnSpeed;

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

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
