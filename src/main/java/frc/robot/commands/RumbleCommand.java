// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;


public class RumbleCommand extends Command {
  //private final ShooterSubsystem m_subsystem;

  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort),
  m_auxController = new PS4Controller(OIConstants.kSecondControllerPort);

  int controllerRumble;
  int leftRight;

  public RumbleCommand(int controller, int dir) {
    // m_subsystem = subsystem;
    // addRequirements(subsystem);
    controllerRumble = controller;
    leftRight = dir;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    if(controllerRumble == 0){
      if(leftRight == 0){
        m_driverController.setRumble(RumbleType.kRightRumble, 1);
      }else if(leftRight == 1){
        m_driverController.setRumble(RumbleType.kLeftRumble, 1);
      }else{
        m_driverController.setRumble(RumbleType.kRightRumble, 1);
        m_driverController.setRumble(RumbleType.kLeftRumble, 1);
      }
    }

    if(controllerRumble == 1){
      if(leftRight == 0){
        m_auxController.setRumble(RumbleType.kRightRumble, 1);
      }else if(leftRight == 1){
        m_auxController.setRumble(RumbleType.kLeftRumble, 1);
      }else{
        m_auxController.setRumble(RumbleType.kRightRumble, 1);
        m_auxController.setRumble(RumbleType.kLeftRumble, 1);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driverController.setRumble(RumbleType.kRightRumble, 0);
    m_driverController.setRumble(RumbleType.kLeftRumble, 0);
    m_auxController.setRumble(RumbleType.kRightRumble, 0);
    m_auxController.setRumble(RumbleType.kLeftRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
