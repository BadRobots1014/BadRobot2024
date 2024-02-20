package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.NavXGyroSubsystem;

public class SwerveDriveCommand extends Command {

    public double m_xSpeed = -512; //negative error code
    public double m_ySpeed = -512;
    public double m_turnSpeed = -512;

    public final SwerveSubsystem swerveSubsystem;
    public final LimelightSubsystem m_LimelightSubsystem;
    //public final NavXGyroSubsystem m_GyroSubsystem;
    public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    public final Supplier<Boolean> fieldOrientedFunction;
    public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public final Supplier<Integer> autoAimMode;
    private final ShuffleboardTab m_AutoAimtab = Shuffleboard.getTab("AutoAim");
    private double AprilTagCurrentID = -1;
    private double currentTargetYaw = 0;
    private double currentYaw = 0;
    private double currentTx = 0;
    private double currentTargetX = 0;
    private double currentTargetZ = 0;
//, NavXGyroSubsystem GyroSubsystem

    public SwerveDriveCommand(SwerveSubsystem subsystem, LimelightSubsystem limelightSubsystem, Supplier<Double> xSupplier, Supplier<Double> ySupplier,
            Supplier<Double> turnSupplier, Supplier<Boolean> fieldOriented, Supplier<Integer> getAutoAimMode) {//all axis are flipped...  right is left.  up is down, clockwise is counterclockwise.
        swerveSubsystem = subsystem;
        m_LimelightSubsystem = limelightSubsystem;
        //m_GyroSubsystem = GyroSubsystem;
        xSpdFunction = xSupplier;
        ySpdFunction = ySupplier;
        turningSpdFunction = turnSupplier;
        fieldOrientedFunction = fieldOriented;
        autoAimMode = getAutoAimMode;
        xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
        yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);

//if(autoAimMode.get() == DriveConstants.kFlexibleAutoAim){
        //m_AutoAimtab.addNumber("Current Tag ID", m_LimelightSubsystem.getAprilTagIDSupplier());
        // m_AutoAimtab.addNumber("Target Yaw:", this::getCurrentTargetYaw);
        // m_AutoAimtab.addNumber("Yaw:", this::getCurrentYaw);
        // m_AutoAimtab.addNumber("X", this::getXSpeed);
        // m_AutoAimtab.addNumber("Y", this::getYSpeed);
        // m_AutoAimtab.addNumber("Turn Speed", this::getTurnSpeed);
//}


        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // Get inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        AprilTagCurrentID = m_LimelightSubsystem.getAprilTagID();
        //currentYaw = m_GyroSubsystem.getYaw();
        currentTx = m_LimelightSubsystem.getTx();
        currentTargetX = m_LimelightSubsystem.getCameraPoseX();
        currentTargetZ = m_LimelightSubsystem.getCameraPoseZ();

        //Insert autoaim and hyjack joystick controls

        //dw Constant police I used constants this time xD 

        if(autoAimMode.get() == DriveConstants.kFlexibleAutoAim){ //Flexible Auto Aim
            //Flexible autoaim will allow the driver to move around the field while button is held while locking the rotation and shooter alignments to point towards the speaker
            if(AprilTagCurrentID != -1){
                currentTargetYaw = m_LimelightSubsystem.getCameraPoseYaw();//set turningspeed while it sees tag
                //need to potentially add something to clear previous yaw if hasent seen tag in a while
            }
            turningSpeed = MathUtil.clamp(0 - currentTx, -1.0, 1.0); //rotate robot to face tag
            //Insert set shooter angle here
            



        }
        if(autoAimMode.get() == DriveConstants.kRigidAutoAim){
            //Rigid autoaim will lock all controls while button is held and keep the robot in a set known position where it is known to have 100% hit rate to fire into speaker
            //TODO: make worky
            if(AprilTagCurrentID != -1){
                currentTargetYaw = m_LimelightSubsystem.getAprilTagPoseYaw();
                
            }
            //turningSpeed = MathUtil.clamp(0.2 * (m_LimelightSubsystem.getAutoAimYawTurnDistance()), -1.0, 1.0); //0.2 of max speed turn
            turningSpeed = MathUtil.clamp((currentTargetYaw/30),-1.0,1.0);
            
            //ySpeed = ; TBD how far to move to

            if (currentTargetX == DriveConstants.kAutoAimSensitivity) {
                xSpeed = 0;
            } else {
                xSpeed = MathUtil.clamp((0 - currentTargetX)/10, -1.0, 1.0);
            }
            //Insert set shooter angle here
        }

        // Death
        xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

        // Slew soup
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleMaxMetersPerSec;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleMaxMetersPerSec;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleMaxRadiansPerSec;

        // I am speed
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get() && autoAimMode.get() == DriveConstants.kAutoAimInactive) {
            // Field oriented
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                    swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        //set var for shuffleboard
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_turnSpeed = turningSpeed;

        // Divide and conker
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Actually do the thing
        swerveSubsystem.setModuleStates(moduleStates);
    }

    public double getXSpeed(){
        return m_xSpeed;
    }
    public double getYSpeed(){
        return m_ySpeed;
    }
    public double getTurnSpeed(){
        return m_turnSpeed;
    }
    public double getCurrentTargetYaw(){
        return currentTargetYaw;
    }
    public double getCurrentYaw(){
        return currentYaw;
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