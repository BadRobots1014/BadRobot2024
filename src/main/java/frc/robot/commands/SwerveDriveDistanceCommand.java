package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;


public class SwerveDriveDistanceCommand extends Command {

  public final SwerveSubsystem swerveSubsystem;
  //public final NavXGyroSubsystem m_gyroSubsystem; oh boy i love eating gyros
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  public final Supplier<Boolean> fieldOrientedFunction, fastModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public final Supplier<Boolean> fieldOriented = ()-> false;
  public final Supplier<Boolean> fastMode = ()-> false;
  public final Supplier<Double> xSupplier = ()-> 0.0;
  public final Supplier<Double> ySupplier = ()-> 0.0;
  public final Supplier<Double> turnSupplier = ()-> 0.0;
  private ShuffleboardTab m_tab;
  private double initial_yaw;
  private double initialX;
  private double initialY;
  private double initialZ;
  private double adjustedInitialX;
  private double adjustedInitialY;
  private double movementHeading;
  private double targetDistance;
  private boolean isDriveFinished = false;

  public SwerveDriveDistanceCommand(
    SwerveSubsystem subsystem,
    //NavXGyroSubsystem gyro,// oh boy i love greek gyros
    double disMeters,
    double movHeadingDegrees
  ) {
    swerveSubsystem = subsystem;
   // m_gyroSubsystem = gyro;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
    fastModeFunction = fastMode;
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    movementHeading = movHeadingDegrees;
    targetDistance = disMeters * 0.05;
    addRequirements(swerveSubsystem);

     //m_tab = Shuffleboard.getTab("AutoDriveDistance");
  }

  @Override
  public void initialize(){
    //swerveSubsystem.resetPose(new Pose2d());
    //swerveSubsystem.setOffset(swerveSubsystem.getPose()); //sets the offset to this pose
    //initial_yaw = swerveSubsystem.getHeading();
    isDriveFinished = false;
    System.out.println("SWERVEDRIVEDISTANCEINIT");
    
    //based on old photo of toecracker bot, it appears that -X is forwards, +X is backwards, +Y is right and -Y is left
    initialX = 0;//swerveSubsystem.getX(); //should be forwards/backwards
    initialY = swerveSubsystem.getFrontLeftDriveDistanceMeters(); //should be right/left based on this image : https://www.google.com/search?client=firefox-b-1-d&sca_esv=2aa9b945258dbd75&sxsrf=ACQVn0_KiBiKJdKv6iHRNoWv4OEuhEhWrg:1708739659604&q=Navx+gyro+displacement+directions&uds=AMwkrPtkV4xyIj1O_U0idwcJ94r1PEfKqwAeYQNHK6u7Wd65vv0Q8q8w72SXjRzgc89eBQfJDzf_M6j9io2l6W1DnNVoZDm7_ahGdixlS7zjPaubzekRtAF30VmD-wSGqiS0YBfIaXTUimbyLlmFpgN5JpVgS8spCw&udm=2&sa=X&ved=2ahUKEwjQgbKj78KEAxVlGtAFHZRWDjEQtKgLegQIBxAB&biw=1920&bih=927&dpr=1#vhid=tqj-ZJSm3KsBwM&vssid=mosaichttps://www.google.com/search?client=firefox-b-1-d&sca_esv=2aa9b945258dbd75&sxsrf=ACQVn0_KiBiKJdKv6iHRNoWv4OEuhEhWrg:1708739659604&q=Navx+gyro+displacement+directions&uds=AMwkrPtkV4xyIj1O_U0idwcJ94r1PEfKqwAeYQNHK6u7Wd65vv0Q8q8w72SXjRzgc89eBQfJDzf_M6j9io2l6W1DnNVoZDm7_ahGdixlS7zjPaubzekRtAF30VmD-wSGqiS0YBfIaXTUimbyLlmFpgN5JpVgS8spCw&udm=2&sa=X&ved=2ahUKEwjQgbKj78KEAxVlGtAFHZRWDjEQtKgLegQIBxAB&biw=1920&bih=927&dpr=1#vhid=tqj-ZJSm3KsBwM&vssid=mosaichttps://www.google.com/search?client=firefox-b-1-d&sca_esv=2aa9b945258dbd75&sxsrf=ACQVn0_KiBiKJdKv6iHRNoWv4OEuhEhWrg:1708739659604&q=Navx+gyro+displacement+directions&uds=AMwkrPtkV4xyIj1O_U0idwcJ94r1PEfKqwAeYQNHK6u7Wd65vv0Q8q8w72SXjRzgc89eBQfJDzf_M6j9io2l6W1DnNVoZDm7_ahGdixlS7zjPaubzekRtAF30VmD-wSGqiS0YBfIaXTUimbyLlmFpgN5JpVgS8spCw&udm=2&sa=X&ved=2ahUKEwjQgbKj78KEAxVlGtAFHZRWDjEQtKgLegQIBxAB&biw=1920&bih=927&dpr=1#vhid=tqj-ZJSm3KsBwM&vssid=mosaichttps://www.google.com/search?client=firefox-b-1-d&sca_esv=2aa9b945258dbd75&sxsrf=ACQVn0_KiBiKJdKv6iHRNoWv4OEuhEhWrg:1708739659604&q=Navx+gyro+displacement+directions&uds=AMwkrPtkV4xyIj1O_U0idwcJ94r1PEfKqwAeYQNHK6u7Wd65vv0Q8q8w72SXjRzgc89eBQfJDzf_M6j9io2l6W1DnNVoZDm7_ahGdixlS7zjPaubzekRtAF30VmD-wSGqiS0YBfIaXTUimbyLlmFpgN5JpVgS8spCw&udm=2&sa=X&ved=2ahUKEwjQgbKj78KEAxVlGtAFHZRWDjEQtKgLegQIBxAB&biw=1920&bih=927&dpr=1#vhid=tqj-ZJSm3KsBwM&vssid=mosaic
    //displacement XY and Z are all in meters
  }

  @Override
  public void execute() {
    //fix axes so not compuzzling
  adjustedInitialX = initialX;      //so should now be +X is right and -X is left
  adjustedInitialY = initialY;  // so +Y should now be forwards and -Y should be back

  double currentDistance = swerveSubsystem.getFrontLeftDriveDistanceMeters();

  //also corrected to make sense
    double currentY = swerveSubsystem.getFrontLeftDriveDistanceMeters();//Y+ is actually back 
    double currentX = 0;//swerveSubsystem.getX();// + x is actually right

    //should calculate the distance it travels if wheels are at an angle
    currentX = Math.toDegrees(Math.sin(Math.toRadians(movementHeading))) * currentDistance;
    currentY = Math.toDegrees(Math.cos(Math.toRadians(movementHeading))) * currentDistance;
    

//offset initial values so they are zero
    
    //autodrivedistance stuffs
    double targetX = (targetDistance*Math.sin(Math.toRadians(movementHeading)) + adjustedInitialX); 
    double targetY = (targetDistance*Math.cos(Math.toRadians(movementHeading)) + adjustedInitialY); //adjust so Y+ is forwards intake
    
    
    System.out.println("Initial X:" + adjustedInitialX);
    System.out.println("Initial Y:" + adjustedInitialY);
    System.out.println("Current X:" + currentX);
    System.out.println("Current Y:" + currentY);
    System.out.println("targetX: " + targetX);  
    System.out.println("targetY: " + targetY);  
    System.out.println("Current Distance" + currentDistance);
    System.out.println("IsDriveFinished" + isDriveFinished);

    double deltaX = (targetX - currentX) * 150;
    double deltaY = (targetY - currentY) * 150; //because scale is so small

    //deltaX = 0;//TODO: DO this for now because I am not calculating horizontal movement

    // if(Math.abs(deltaY) <= 0.005){ //may need to adjust how sensitive it is  Math.abs(deltaX) <= 0.005 &&
    //   isDriveFinished = true;
    // }
    

    if(Math.abs(currentDistance) <= 0.005){
      isDriveFinished = true; //stop robot if robot reaches target distance reguardless of direction it thinks it traveled
    }

    if((Math.abs(deltaY) >= (targetY - initialY) + 1) || (Math.abs(deltaX) >= (targetX - initialX) + 1)){
      //prevent runoffs if the inversion of axes is wrong and robot runs more than a meter in the wrong direction
      //can be removed later once confirmed it works
      isDriveFinished = true;
    }



    // Get inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fastMode = fastModeFunction.get();

    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDriveDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDriveDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDriveDeadband ? turningSpeed : 0;

    //Hyjack joysticks

    xSpeed = MathUtil.clamp(deltaX, -0.2, 0.2); //limiting robot to 0.2 for now because I enjoy having unbroken shins
    ySpeed = MathUtil.clamp(deltaY, -0.2, 0.2);
    turningSpeed = 0;

    // xSpeed = MathUtil.clamp(deltaX, -1.0, 1.0);
    // ySpeed = MathUtil.clamp(deltaY, -1.0, 1.0);

//turningSpeed = MathUtil.clamp(speed, -1.0,1.0);


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
    return isDriveFinished; 
  }

  
}
