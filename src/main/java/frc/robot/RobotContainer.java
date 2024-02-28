// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.UpdatePIDCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.commands.auto.ShootAndDriveAutoCommand;
import frc.robot.commands.auto.TurnAndShootAutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.BrakeClimbersCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReleaseClimbersCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterCommand;

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
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // Auto
  private final ShuffleboardTab m_tab;
  private SendableChooser<Command> m_chosenAuto = new SendableChooser<>();

  private boolean fieldOriented = DriveConstants.kFieldOriented;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_robotDrive.setDefaultCommand(new SwerveDriveCommand(
            m_robotDrive,
            () -> Math.pow(getLeftX(), 3),
            () -> Math.pow(getLeftY(), 3),
            () -> Math.pow(getRightX(), 3),
            this::getFieldOriented,
            this::getFastMode));

    // Auto chooser setup
    m_tab = Shuffleboard.getTab("Auto");

    m_chosenAuto.setDefaultOption("Shoot and drive from middle",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d()));
    m_chosenAuto.addOption("Shoot and drive from left",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(45))));
    m_chosenAuto.addOption("Shoot and drive from right",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(-45))));
    m_chosenAuto.addOption("Drive, turn, and shoot from left",
      new TurnAndShootAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(), 40));
    m_chosenAuto.addOption("Drive, turn, and shoot from right",
      new TurnAndShootAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(), -40));

    m_tab.add(m_chosenAuto);

    // Configure the button bindings
    configureButtonBindings();
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
    new JoystickButton(m_driverController, XboxController.Button.kBack.value) // Reset gyro
      .whileTrue(new ZeroHeadingCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kY.value) // TODO Autoaim
      .whileTrue(new UpdatePIDCommand(m_robotDrive));
      //                                                       kB.value) // Toggle fastmode
      //                                                       kStart.value) // Toggle field oriented

    // Auxillary stuff
    new JoystickButton(m_auxController, XboxController.Button.kRightBumper.value) // Shoot
      .whileTrue(new ShootCommand(m_shooterSubsystem));
    new JoystickButton(m_auxController, XboxController.Button.kLeftBumper.value) // Intake
      .whileTrue(new IntakeCommand(m_shooterSubsystem));
    new JoystickButton(m_auxController, XboxController.Button.kX.value) // Winch up
      .whileTrue(new ShooterCommand(m_shooterSubsystem, "winch up"));
    new JoystickButton(m_auxController, XboxController.Button.kA.value) // Winch down
      .whileTrue(new ShooterCommand(m_shooterSubsystem, "winch down"));
    new JoystickButton(m_auxController, XboxController.Button.kY.value) // Climber up
      .whileTrue(new ClimbCommand(m_climberSubsystem, .5));
    new JoystickButton(m_auxController, XboxController.Button.kB.value) // Climber down
      .whileTrue(new ClimbCommand(m_climberSubsystem, -.5));
    new JoystickButton(m_auxController, XboxController.Button.kBack.value) // Drop climbers (they go up)
      .whileTrue(new ReleaseClimbersCommand(m_climberSubsystem));
    new JoystickButton(m_auxController, XboxController.Button.kStart.value) // Brake climbers (they stop)
      .whileTrue(new BrakeClimbersCommand(m_climberSubsystem));
  }

  boolean getFastMode() {
    if (m_driverController.getBButtonPressed()) {
      fastMode = !fastMode;
    }
    return fastMode;
  }

  boolean getFieldOriented() {
    if (m_driverController.getStartButtonPressed()) {
      fieldOriented = !fieldOriented;
    }
    return fieldOriented;
  }

  double getRightX() {return m_driverController.getRightX();}
  double getLeftX() {return -m_driverController.getLeftX();}
  double getLeftY() {return -m_driverController.getLeftY();}
  double getPOV() {return m_driverController.getPOV();}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chosenAuto.getSelected();
  }
}
