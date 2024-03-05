// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax leftClimber;
  private final CANSparkMax rightClimber;
  private final ShuffleboardTab m_tab;

  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberCanId, MotorType.kBrushed);
    rightClimber = new CANSparkMax(ClimberConstants.kRightClimberCanId, MotorType.kBrushed);
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    m_tab = Shuffleboard.getTab("Climbers");
    m_tab.addNumber("Left climber amps", this::getLeftClimberCurrent);
    m_tab.addNumber("Right climber amps", this::getRightClimberCurrent);
  }

  public void runClimbers(double power) {
    runClimber(false, power);
    runClimber(true, power);
  }

  public void runClimber(boolean right, double power) {
    if (right) rightClimber.set(getLeftClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
    else leftClimber.set(getRightClimberCurrent() < ClimberConstants.kClimberMaxAmps ? power : 0);
  }

  public void stopClimbers() {
    stopClimber(false);
    stopClimber(true);
  }

  public void stopClimber(boolean right) {
    if (right) rightClimber.stopMotor();
    else leftClimber.stopMotor();
  }

  public void setClimberIdleMode(IdleMode rightMode, IdleMode leftMode) {
    leftClimber.setIdleMode(leftMode);
    rightClimber.setIdleMode(rightMode);
  }

  public double getLeftClimberCurrent() {return leftClimber.getOutputCurrent();}
  public double getRightClimberCurrent() {return rightClimber.getOutputCurrent();}

}
