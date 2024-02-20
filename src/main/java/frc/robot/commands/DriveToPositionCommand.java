package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class DriveToPositionCommand extends Command {

  public final SwerveSubsystem swerve;
  public final SwerveDriveCommand driveCommand;
  public final Supplier<Double> goalX, goalY, goalAngle, xSpdFunction, ySpdFunction, turningSpdFunction;
  public final Supplier<Boolean> fieldOrientedFunction, fastModeFunction, reset;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public DriveToPositionCommand(
    SwerveSubsystem subsystem,
    Supplier<Double> goalX,
    Supplier<Double> goalY,
    Supplier<Double> goalAngle,
    Supplier<Boolean> fieldOriented,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> resetOnStart
  ) {
    swerve = subsystem;
    this.goalX = goalX;
    this.goalY = goalY;
    this.goalAngle = goalAngle;
    this.xSpdFunction = goalX;
    this.ySpdFunction = goalY;
    this.turningSpdFunction = goalAngle;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    reset = resetOnStart;
    driveCommand = new SwerveDriveCommand(subsystem, xSpdFunction, ySpdFunction, turningSpdFunction, fieldOriented, fastMode);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (reset.get()) {
      swerve.resetPose(new Pose2d());
    }
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
