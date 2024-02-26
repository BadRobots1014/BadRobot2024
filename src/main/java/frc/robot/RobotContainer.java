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
import frc.robot.commands.auto.DriveDistanceAtAngleAutoCommand;
import frc.robot.commands.auto.DriveDistanceAutoCommand;
import frc.robot.commands.auto.ShootAndDriveAutoCommand;
import frc.robot.commands.auto.TurnThetaAutoCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);


  
  // Shooter Subsystem
  // Subsystems
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem(m_driverController);
  private boolean fastMode = false;

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


    //Auto Selector
    m_tab = Shuffleboard.getTab("Auto");

    m_chosenAuto.setDefaultOption("Shoot and drive middle",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d()));
    m_chosenAuto.addOption("Shoot and drive left",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(45))));
    m_chosenAuto.addOption("Shoot and drive right",
      new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(-45))));

      m_chosenAuto.addOption("Shinbreak 0.2 meters forwards at 45 degree movement heading", 
    new DriveDistanceAtAngleAutoCommand(m_robotDrive));
    m_chosenAuto.addOption("Shinbreak 0.2 meters forwards", 
    new DriveDistanceAutoCommand(m_robotDrive));
    m_chosenAuto.addOption("Shinbreak 45 degrees left", 
    new TurnThetaAutoCommand(m_robotDrive));

    m_tab.add(m_chosenAuto);

    // Configure the button bindings
    configureButtonBindings();

    // //Auto Commands
    // this.m_driveDistanceAuto = new DriveDistanceAutoCommand(m_robotDrive, m_GyroSubsystem);
    // this.m_turnThetaAuto = new TurnThetaAutoCommand(m_robotDrive, m_GyroSubsystem);
    // this.m_shootAndDriveAuto = new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive);
    
    // //Auto Selector
    // m_AutoSelector.addOption("Shinbreak 1 meter at 45 degrees right", m_driveDistanceAuto);
    // m_AutoSelector.addOption("Shinbreak 45 degrees left", m_turnThetaAuto);
    // m_AutoSelector.addOption("Shoot and Drive Auto", m_shootAndDriveAuto);

    // //Auto Shuffleboard Selector
    // Shuffleboard.getTab("Auto Selector").add("Auto Selector Panel", m_AutoSelector);
    
    

    // Setup paths
    //m_autoPath = PathPlannerPath.fromPathFile("New Path");
    // m_autoTraj = new PathPlannerTrajectory(m_autoPath,
    // m_robotDrive.getModuleStates(), m_robotDrive.getRotation2d());
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
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .whileTrue(new ZeroHeadingCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new UpdatePIDCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new ShootCommand(m_shooterSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new IntakeCommand(m_shooterSubsystem));
  }

  double getRightX() {
    return m_driverController.getRightX();
  }

  double getLeftX() {
    var pov = m_driverController.getPOV();
    if (pov > -1) {
      if (pov == 90)
        return 1;
      if (pov == 270)
        return -1;
    }

    return -m_driverController.getLeftX();
  }

  double getLeftY() {
    int pov = m_driverController.getPOV();

    if (pov > -1) {
      if (pov == 0)
        return -1;
      if (pov == 180)
        return 1;
    }

    return -m_driverController.getLeftY();
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new ShootAndDriveAutoCommand(m_shooterSubsystem, m_robotDrive);
    return m_chosenAuto.getSelected();
  }
}
