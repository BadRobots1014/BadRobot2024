// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TestMotorCommand;
import frc.robot.commands.UpdatePIDCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.SwerveSubsystem;

// Shooter
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_rightJoystick = new Joystick(0);
  Joystick m_leftJoystick = new Joystick(1);

  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem(m_driverController);

  // Shooter Subsystem
  private final ShooterSubsystem m_shooterarmsystem = new ShooterSubsystem(0.0);

  // Paths
  private PathPlannerTrajectory m_autoTraj;
  private PathPlannerPath m_autoPath;
  private PathPlannerAuto m_auto;

  // TEST
  private double m_testMotorId = 0;
  private double m_testMotorSpeed = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive.setDefaultCommand(new SwerveDriveCommand(
        m_robotDrive,
        () -> Math.pow(getLeftX(), 3),
        () -> Math.pow(getLeftY(), 3),
        () -> Math.pow(getRightX(), 3),
        () -> DriveConstants.kFieldOriented));

    // m_robotDrive.setDefaultCommand();

    // Configure the button bindings
    configureButtonBindings();

    // Setup paths
    m_autoPath = PathPlannerPath.fromPathFile("New Path");
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
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new ZeroHeadingCommand(m_robotDrive));
    new JoystickButton(m_driverController, DriveConstants.kTestMotorButton.value)
        .whileTrue(new TestMotorCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new UpdatePIDCommand(m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new ShooterCommand(m_shooterarmsystem, ShooterCommand.CommandType.StartMotor));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .onTrue(new ShooterCommand(m_shooterarmsystem, ShooterCommand.CommandType.Shoot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_auto = new PathPlannerAuto("New Auto");
    return m_auto;
    // return m_robotDrive.followTrajectoryCommand(m_autoTraj, m_autoPath, true);
  }

  double getRightX() {
    if (Math.abs(m_driverController.getRightX()) < Constants.DriveConstants.kJoystickDeadzone)
      return 0;
    return -m_driverController.getRightX();
  }

  double getLeftX() {
    int pov = m_driverController.getPOV();

    if (pov > -1) {
      if (pov == 90)
        return 1;
      if (pov == 270)
        return -1;
    }

    if (Math.abs(m_driverController.getLeftX()) < Constants.DriveConstants.kJoystickDeadzone)
      return 0;
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

    if (Math.abs(m_driverController.getLeftY()) < Constants.DriveConstants.kJoystickDeadzone)
      return 0;
    return -m_driverController.getLeftY();

  }
}