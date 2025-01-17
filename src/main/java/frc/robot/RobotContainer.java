// Copyright  (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.WinchPresetCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.auto.DriveAutoCommand;
import frc.robot.commands.auto.ShootAndDriveAutoCommand;
import frc.robot.commands.auto.ShootAutoCommand;
import frc.robot.commands.auto.TurnAndShootAutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AirIntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ExpelRingCommand;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.GroundIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetWinchCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShootCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_auxController = new XboxController(OIConstants.kSecondControllerPort);

  // Subsystems
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem();
  private boolean fastMode = false;
  private boolean fasterMode = false;
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Auto
  private final ShuffleboardTab m_tab;
  private SendableChooser<Command> m_chosenAuto = new SendableChooser<>();
  private GenericEntry m_delay;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_robotDrive.setDefaultCommand(new SwerveDriveCommand(
            m_robotDrive,
            () -> getLeftX(),
            () -> getLeftY(),
            () -> getRightX(),
            DriveConstants.kFieldOriented,
            this::getFastMode,
            this::getFasterMode,
            this::getPOV));
    m_climberSubsystem.setDefaultCommand(new ClimbCommand(m_climberSubsystem, this::getAuxRightY, this::getAuxLeftY));
    m_shooterSubsystem.setDefaultCommand(new WinchCommand(m_shooterSubsystem, this::POVToWinchSpeed));
    m_intakeSubsystem.setDefaultCommand(new RetractIntakeCommand(m_intakeSubsystem));

    // Auto chooser setup
    m_tab = Shuffleboard.getTab("Auto");

    m_delay = m_tab.add("Delay", 0).getEntry();

    m_chosenAuto.setDefaultOption("Shoot and drive from middle",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0,0,Rotation2d.fromDegrees(0)), m_delay.getDouble(0))); //used to be an empty Pose2D (should not change but maybe the empty pose has something to do with it)
    m_chosenAuto.addOption("Shoot and drive from left",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(55)), m_delay.getDouble(0)));
    m_chosenAuto.addOption("Shoot and drive from right",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(-55)), m_delay.getDouble(0)));
    m_chosenAuto.addOption("Drive, turn, and shoot from left",
      new TurnAndShootAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(), 55, m_delay.getDouble(0)));
    m_chosenAuto.addOption("Drive, turn, and shoot from right",
      new TurnAndShootAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(), -55, m_delay.getDouble(0)));
    m_chosenAuto.addOption("Drive back only",
      new DriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0,0,Rotation2d.fromDegrees(0)), m_delay.getDouble(0))); //drive back only auto
    m_chosenAuto.addOption("Shoot only",
      new ShootAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(), m_delay.getDouble(0)).withTimeout(4));

    m_tab.add(m_chosenAuto);

    // Configure the button bindings
    configureButtonBindings();
  }

  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver stuff
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value) // Reset gyro
      .whileTrue(new ZeroHeadingCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new GroundIntakeCommand(m_intakeSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new ExpelRingCommand(m_intakeSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new FeedShooterCommand(m_intakeSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new AirIntakeCommand(m_intakeSubsystem));
      // Left bumper = Toggle fastmode
      // Left trigger = Toggle fastermode
      // POV = Nudge
      // Right joystick = Move
      // Left joystick = Turn

    // Auxillary stuff
    new JoystickButton(m_auxController, XboxController.Button.kRightBumper.value) // Shoot
      .whileTrue(new ShootCommand(m_shooterSubsystem));
    new JoystickButton(m_auxController, XboxController.Button.kLeftBumper.value) // Intake
      .whileTrue(new IntakeCommand(m_shooterSubsystem));
    new JoystickButton(m_auxController, XboxController.Button.kB.value) // Climber up
      .whileTrue(new ClimbCommand(m_climberSubsystem, ClimberConstants.kClimberUpPower));
    new JoystickButton(m_auxController, XboxController.Button.kA.value) // Climber down
      .whileTrue(new ClimbCommand(m_climberSubsystem, ClimberConstants.kClimberDownPower));
    new JoystickButton(m_auxController, XboxController.Button.kY.value) // Winch up preset
      .whileTrue(new WinchPresetCommand(m_shooterSubsystem, ShooterConstants.kWinchUpPreset));
    new JoystickButton(m_auxController, XboxController.Button.kX.value) // Winch down preset
      .whileTrue(new WinchPresetCommand(m_shooterSubsystem, ShooterConstants.kWinchDownPreset));
    new JoystickButton(m_auxController, XboxController.Button.kBack.value) // Reset winch encoder
      .whileTrue(new ResetWinchCommand(m_shooterSubsystem));
      // POV = Winch
      // Joysticks = Manual climbers
  }

  boolean getFastMode() {
    if (m_driverController.getLeftBumperPressed()) {
      fastMode = !fastMode;
    }
    return fastMode;
  }

  boolean getFasterMode() {
    if (m_driverController.getLeftTriggerAxis() > OIConstants.kTriggerDeadband) {
      fasterMode = true;
    }
    else fasterMode = false;
    return fasterMode;
  }
  double getRightX() {return m_driverController.getRightX();}
  double getLeftX() {return -m_driverController.getLeftX();}
  double getLeftY() {return -m_driverController.getLeftY();}
  double getPOV() {return m_driverController.getPOV();}
  double getAuxRightY() {return Math.abs(m_auxController.getRightY()) > OIConstants.kDriveDeadband ? m_auxController.getRightY() : 0;}
  double getAuxLeftY() {return Math.abs(m_auxController.getLeftY()) > OIConstants.kDriveDeadband ? m_auxController.getLeftY() : 0;}
  double getAuxPOV() {return m_auxController.getPOV();}
  double POVToWinchSpeed() {
    return getAuxPOV() == 0 ? ShooterConstants.kWinchUpPower : (getAuxPOV() == 180 ? ShooterConstants.kWinchDownPower : 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chosenAuto.getSelected();
  }
}
