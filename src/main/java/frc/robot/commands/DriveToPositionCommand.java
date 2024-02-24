package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
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
    //The speed the wheels run at should be proportional to the distance to the goal point (desaturated to a more reasonable speed later)
    this.xSpdFunction = new Supplier<Double>() {
      @Override
      public Double get() {
        return goalX.get() - swerve.getX();
      }
    };
    this.ySpdFunction = new Supplier<Double>() {
      @Override
      public Double get() {
        return goalY.get() - swerve.getY();
      }
    };
    this.turningSpdFunction = new Supplier<Double>() {
      @Override
      public Double get() {
        return (goalAngle.get() - swerve.getHeading()) / 360;
      }
    };
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
      swerve.resetPose();
    }
  }

  @Override
  public void execute() {
    driveCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
