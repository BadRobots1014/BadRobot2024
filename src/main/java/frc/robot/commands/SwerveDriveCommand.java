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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveDriveCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  public final LimelightSubsystem m_LimelightSubsystem;
  public final ShooterSubsystem m_ShooterSubsystem;

  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, pov;
  public Supplier<Boolean> fastModeFunction, fasterModeFunction;
  Supplier<Integer> autoAimMode;

    private double AprilTagCurrentID = -1;
    private double currentTargetYaw = 0;
    private double currentYaw = 0;
    private double currentTx = 0;
    private double currentTargetX = 0;
    private double currentTargetZ = 0;
    Supplier<Double> winchPosSupplier = ()-> 0.0;
    WinchPresetCommand m_WinchRunToPosition;
    

  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public boolean fieldOrientedFunction;
  private ShuffleboardTab m_tab;
  private GenericEntry shuffleFieldOriented;

  public SwerveDriveCommand(
    SwerveSubsystem subsystem,
    LimelightSubsystem limelight,
    ShooterSubsystem shooter,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    boolean fieldOriented,
    Supplier<Boolean> fastMode,
    Supplier<Boolean> fasterMode,
    Supplier<Double> povSupplier,
    Supplier<Integer> getAutoAimMode
  ) {
    swerveSubsystem = subsystem;
    m_LimelightSubsystem = limelight;
    m_ShooterSubsystem = shooter;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    fasterModeFunction = fasterMode;
    pov = povSupplier;
    autoAimMode = getAutoAimMode;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);

    m_WinchRunToPosition = new WinchPresetCommand(m_ShooterSubsystem, winchPosSupplier);

    m_tab = Shuffleboard.getTab("Swerve Instance");
    shuffleFieldOriented = m_tab.add("Field Oriented" + this.toString(), fieldOriented).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  }

  @Override
  public void execute() {
    // Get inputs
    double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
    boolean fastMode = false, fasterMode = false;
    AprilTagCurrentID = m_LimelightSubsystem.getAprilTagID();
    //currentYaw = m_GyroSubsystem.getYaw();
    currentTx = m_LimelightSubsystem.getTx();
    currentTargetX = m_LimelightSubsystem.getCameraPoseX();
    currentTargetZ = m_LimelightSubsystem.getCameraPoseZ();

    if (pov.get() == -1 && autoAimMode.get() == DriveConstants.kAutoAimInactive) {
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
      fastMode = fastModeFunction.get();
      fasterMode = fasterModeFunction.get();
    }else if(autoAimMode.get() == DriveConstants.kFlexibleAutoAim){
        //Flexible autoaim will allow the driver to move around the field while button is held while locking the rotation and shooter alignments to point towards the speaker
            if(AprilTagCurrentID == 3){
              m_LimelightSubsystem.setPriorityID(3);//ID 3 and 7
            }
            if(AprilTagCurrentID ==7){
              m_LimelightSubsystem.setPriorityID(7);//ID 3 and 7
            }

            
            m_LimelightSubsystem.setPriorityID(3);//ID 3 and 7
            currentTx = m_LimelightSubsystem.getTx();
                //currentTargetYaw = m_LimelightSubsystem.getCameraPoseYaw();//set turningspeed while it sees tag
                //need to potentially add something to clear previous yaw if hasent seen tag in a while
            turningSpeed = MathUtil.clamp((0 - currentTx)/80, -1.0, 1.0); //rotate robot to face tag

            
            double shooterSetAngle = m_ShooterSubsystem.angleToEncoderCounts(m_LimelightSubsystem.getAutoAimShooterAngle());
            double winchSetPos = MathUtil.clamp(shooterSetAngle, ShooterConstants.kWinchUpPreset, ShooterConstants.kWinchDownPreset);

            winchPosSupplier = () -> winchSetPos;

            //Insert set shooter angle here
            
            m_WinchRunToPosition.schedule();

            m_LimelightSubsystem.resetPriorityID();
    }else if(autoAimMode.get() == DriveConstants.kRigidAutoAim){//This will now be for source autoaim
            if(AprilTagCurrentID == 1){
              m_LimelightSubsystem.setPriorityID(1);//ID 1 and 9
            }
            if(AprilTagCurrentID ==9){
              m_LimelightSubsystem.setPriorityID(9);//ID 1 and 9
            }
            currentTx = m_LimelightSubsystem.getTx();
            currentTargetX = m_LimelightSubsystem.getCameraPoseX();
            currentTargetZ = m_LimelightSubsystem.getCameraPoseZ();

            
            xSpeed = MathUtil.clamp((currentTargetX + DriveConstants.kAutoAimXOffset)/8, -0.3, 0.3); //clamped to 0.5 speed max
            ySpeed = MathUtil.clamp((currentTargetZ + DriveConstants.kAutoAimZOffset)/8,-0.3,0.3);
            turningSpeed = MathUtil.clamp((0 - currentTx)/80, -1.0, 1.0); //rotate robot to face tag

            //Winch will be manual for now

            m_LimelightSubsystem.resetPriorityID();
    }
    else {
      xSpeed = pov.get() == 90 ? -DriveConstants.kNudgeSpeed : (pov.get() == 270 ? DriveConstants.kNudgeSpeed : 0);
      ySpeed = pov.get() == 0 ? DriveConstants.kNudgeSpeed : (pov.get() == 180 ? -DriveConstants.kNudgeSpeed : 0);
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

  private void WinchPresetCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'WinchPresetCommand'");
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
