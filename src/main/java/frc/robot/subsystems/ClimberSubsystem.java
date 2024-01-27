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
import frc.robot.Constants.CLimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushed);
    CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);

    ShuffleboardTab m_climbTab = Shuffleboard.getTab("climber");

    NavXGyroSubsystem m_Subsystem;
  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem(NavXGyroSubsystem subsystem) {
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_climbTab.addDouble("left", m_leftMotor::get);
    m_climbTab.addDouble("right", m_rightMotor::get);
  }

  public void climb()
  {
    double roll = m_Subsystem.getRoll();

    if (roll < -CLimberConstants.NavXClimberDeadzone)
    {
        m_rightMotor.set(CLimberConstants.ClimbSpeed);
    }
    else if (roll > CLimberConstants.NavXClimberDeadzone)
    {
        m_leftMotor.set(CLimberConstants.ClimbSpeed);
    }
    else
    {
        m_rightMotor.set(CLimberConstants.ClimbSpeed);
        m_leftMotor.set(CLimberConstants.ClimbSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}