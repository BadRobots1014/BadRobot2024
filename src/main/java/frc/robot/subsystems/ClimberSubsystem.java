// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax leftClimber;
  private final CANSparkMax rightClimber;

  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberCanId, MotorType.kBrushed);
    rightClimber = new CANSparkMax(ClimberConstants.kRightClimberCanId, MotorType.kBrushed);
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);
  }

  public void runClimbers(double power) {
    runClimber(false, power);
    runClimber(true, power);
  }

  public void runClimber(boolean right, double power) {
    if (right) rightClimber.set(power); else leftClimber.set(power);
  }

  public void stopClimbers() {
    stopClimber(false);
    stopClimber(true);
  }

  public void stopClimber(boolean right) {
    if (right) rightClimber.stopMotor(); else leftClimber.stopMotor();
  }

  public void setClimberIdleMode(IdleMode rightMode, IdleMode leftMode) {
    leftClimber.setIdleMode(leftMode);
    rightClimber.setIdleMode(rightMode);
  }
}
