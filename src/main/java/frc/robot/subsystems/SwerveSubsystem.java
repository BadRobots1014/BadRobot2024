package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  XboxController Controller;

  public GenericEntry p;
  public GenericEntry i;
  public GenericEntry d;

  public double offsetX = 0;
  public double offsetY = 0;

  public SwerveSubsystem(XboxController controller) {
    m_tab = Shuffleboard.getTab("swerve");
    Controller = controller;

    p = m_tab.add("p", ModuleConstants.kTurningP).getEntry();
    i = m_tab.add("i", ModuleConstants.kTurningI).getEntry();
    d = m_tab.add("d", ModuleConstants.kTurningD).getEntry();
  }

  // Modules
  public SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftEncoderCanId,
    DriveConstants.kFrontLeftChassisAngularOffset,
    DriveConstants.kFrontLeftAbsoluteEncoderReversed
  );

  public SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightEncoderCanId,
    DriveConstants.kFrontRightChassisAngularOffset,
    DriveConstants.kFrontRightAbsoluteEncoderReversed
  );

  public SwerveModule backLeft = new SwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kRearLeftEncoderCanId,
    DriveConstants.kBackLeftChassisAngularOffset,
    DriveConstants.kBackLeftAbsoluteEncoderReversed
  );

  public SwerveModule backRight = new SwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kRearRightEncoderCanId,
    DriveConstants.kBackRightChassisAngularOffset,
    DriveConstants.kBackRightAbsoluteEncoderReversed
  );

  // The gyro
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Shuffleboard
  private final ShuffleboardTab m_tab;

  public SwerveSubsystem() {
    //Delay to allow navx to boot up
    new Thread(() -> {
      try {
        Thread.sleep(DriveConstants.kBootupDelay);
        resetPose();
      } catch (Exception e) {}
    }).start();

    //Shuffle bored
    m_tab = Shuffleboard.getTab("Swerve");
    m_tab.addNumber("Heading", this::getHeading);
    m_tab.addNumber("Yaw", this::getYaw);
    m_tab.addNumber("Roll", this::getRoll);
    m_tab.addNumber("Pitch", this::getPitch);
    m_tab.addNumber("X", this::getX);
    m_tab.addNumber("Y", this::getY);
    m_tab.addNumber("X Offset", () -> offsetX);
    m_tab.addNumber("Y Offset", () -> offsetY);
    m_tab.addBoolean("NavX isConnected", gyro::isConnected);
    m_tab.addBoolean("NavX isCalibrating", gyro::isCalibrating);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.kAutoConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
      );
  }

  // Gyro data shenanigans
  public void resetPose() {
    gyro.reset();
    gyro.resetDisplacement();
    setOffset(new Pose2d());
  }
  public void resetPose(Pose2d pose) {
    gyro.reset();
    gyro.resetDisplacement();
    setOffset(pose);
  }
  public void setOffset(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    offsetX = pose.getX();
    offsetY = pose.getY();
  }
  public double getHeading() {return Math.IEEEremainder(gyro.getAngle(), 360);}
  public Rotation2d getRotation2d() {return Rotation2d.fromDegrees(getHeading());}
  public double getYaw() {return Math.IEEEremainder(gyro.getYaw(), 360);}
  public double getRoll() {return Math.IEEEremainder(gyro.getRoll(), 360);}
  public double getPitch() {return Math.IEEEremainder(gyro.getPitch(), 360);}
  public double getX() {return gyro.getDisplacementX() + offsetX;}
  public double getY() {return gyro.getDisplacementY() + offsetY;}
  public double getXSpeed() {return gyro.getVelocityX();}
  public double getYSpeed() {return gyro.getVelocityY();}
  public double getTurnSpeed() {return gyro.getRate();}
  public Pose2d getPose() {return new Pose2d(getX(), getY(), getRotation2d());}

  public ChassisSpeeds getRobotRelativeSpeeds() {return new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurnSpeed());}
  public void driveRobotRelative(ChassisSpeeds speeds) {
      setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
  }


  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * @param desiredStates The states the modules should move toward. In order,
   *                      front left, front right, back left, back right.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      DriveConstants.kFastTeleMaxMetersPerSec
    );
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void testMotor() {
    System.out.println(Controller.getPOV());
    m_tab.addInteger("POV", Controller::getPOV);

    if (Controller.getPOV() > -1) {
      int pov = Controller.getPOV();
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
      backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(pov)));
    }

    double speedMultiplyer = Controller.getRightBumper() ? .3 : 1;

    SwerveModule module = null;

    if (Controller.getYButton()) module = frontRight;
    else if (Controller.getXButton()) module = frontLeft;
    else if (Controller.getBButton()) module = backRight;
    else if (Controller.getAButton()) module = backLeft;

    if (module == null) {
      frontLeft.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      frontRight.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      backLeft.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      backRight.setDesiredState(
        new SwerveModuleState(
          Controller.getLeftY() * speedMultiplyer,
          Rotation2d.fromRotations(.5 * Controller.getLeftX())
        )
      );
      return;
    }

    frontLeft.setDesiredState(new SwerveModuleState());
    frontRight.setDesiredState(new SwerveModuleState());
    backLeft.setDesiredState(new SwerveModuleState());
    backRight.setDesiredState(new SwerveModuleState());

    module.setDesiredState(
      new SwerveModuleState(
        Controller.getLeftY() * speedMultiplyer,
        Rotation2d.fromRotations(.5 * Controller.getLeftX())
      )
    );
  }
}
