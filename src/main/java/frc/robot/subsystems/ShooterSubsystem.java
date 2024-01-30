// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShooterSubsystem extends SubsystemBase {
    public final CANSparkMax m_leftMotor;
    public final CANSparkMax m_rightMotor;

    private final ShuffleboardTab m_tab  = Shuffleboard.getTab("Shooter"); // Assuming this will be needed later??

  public ShooterSubsystem() {
    m_leftMotor = new CANSparkMax(ShooterConstants.kRightDeviceId, MotorType.kBrushless); // Assuming brushless? 
    m_rightMotor = new CANSparkMax(ShooterConstants.kLeftDeviceId, MotorType.kBrushless);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void runShooter(double power) {
    m_leftMotor.set(clampPower(power));
    m_rightMotor.set(clampPower(power));
  }
  public void stopMotors(CANSparkMax MotorLeft, CANSparkMax MotorRight) {
    MotorLeft.stopMotor();
    MotorRight.stopMotor();
  }
  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }
}