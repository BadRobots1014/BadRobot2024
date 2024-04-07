// POSITIVE COUNTERCLOCKWISE
// NEGATIVE CLOCKWISE
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Shooter");

  private final GenericEntry m_frontMotorPower, m_backMotorPower, m_frontIntakePower, m_backIntakePower;

  public final CANSparkFlex m_frontMotor, m_backMotor;
  public final SparkPIDController m_frontPID, m_backPID;


  public ShooterSubsystem() {

    m_frontMotor = new CANSparkFlex(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_backMotor = new CANSparkFlex(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
    m_frontMotor.setInverted(false);
    m_backMotor.setInverted(false);
    m_frontPID = m_frontMotor.getPIDController();
    m_backPID = m_backMotor.getPIDController();
    m_frontPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    m_backPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    m_frontMotor.setIdleMode(IdleMode.kCoast);
    m_backMotor.setIdleMode(IdleMode.kCoast);
    m_frontMotor.setInverted(false);
    m_backMotor.setInverted(false);


    // Displays whether or not the shooter is running
    m_shuffleboardtab.addBoolean("Shooter Running", this::isShooterRunning);

    // Shuffleboard numbers for various motor powers
    m_frontMotorPower = m_shuffleboardtab.add("Front Motor Power", ShooterConstants.kFrontShootPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_backMotorPower = m_shuffleboardtab.add("Back Motor Power", ShooterConstants.kBackShootPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_frontIntakePower = m_shuffleboardtab.add("Front Intake Power", ShooterConstants.kFrontIntakePower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_backIntakePower = m_shuffleboardtab.add("Back Intake Power", ShooterConstants.kBackIntakePower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();

    m_shuffleboardtab.addBoolean("Shooter spun up", this::getSpunUp);
    m_shuffleboardtab.addNumber("Front motor RPM", this::getFrontRPM);
    m_shuffleboardtab.addNumber("Back motor RPM", this::getBackRPM);
  }

  // Shooter stuff
  public double[] getShooterPowers() { // Function to get the motor powers from shuffleboard and clamp them to a value between -1 and 1
    return new double[] {
      clampPower(m_frontMotorPower.getDouble(ShooterConstants.kFrontShootPower)*ShooterConstants.kShooterMaxSpeed),
      clampPower(m_backMotorPower.getDouble(ShooterConstants.kBackShootPower)*ShooterConstants.kShooterMaxSpeed)
    };
  }
  public double[] getIntakePowers() {
    return new double[] {
      clampPower(m_frontIntakePower.getDouble(ShooterConstants.kFrontIntakePower)*ShooterConstants.kShooterMaxSpeed),
      clampPower(m_backIntakePower.getDouble(ShooterConstants.kBackIntakePower)*ShooterConstants.kShooterMaxSpeed)
      // clampPower(m_indexIntakePower.getDouble(ShooterConstants.kIndexIntakePower))
    };
  }
  public void runShooter() {
    m_frontMotor.set(getShooterPowers()[0]);
    m_backMotor.set(getShooterPowers()[1]);
  }
  public void runShooter(double rearPower) {
    m_frontMotor.set(getShooterPowers()[0]);
    m_backMotor.set(rearPower*ShooterConstants.kShooterMaxSpeed);
  }
  public void runIntake() {
    m_frontMotor.set(getIntakePowers()[0]);
    m_backMotor.set(getIntakePowers()[1]);
    // m_indexMotor.set(getIntakePowers()[2]);
  }
  public void stopShooter() {
    m_frontMotor.stopMotor();
    m_backMotor.stopMotor();
    // m_indexMotor.stopMotor();
  }
  public boolean isShooterRunning() {
    if (m_frontMotor.get() == 0 && m_backMotor.get() == 0) { //Check if both motors are stopped
      return false; //If so, the shooter is not running
    } else {
      return true; //Otherwise, the shooter is running
    }
  }
  public double getFrontRPM() {
    return m_frontMotor.getEncoder().getVelocity();
  }
  public double getBackRPM() {
    return m_backMotor.getEncoder().getVelocity();
  }
  public boolean getSpunUp() {
    return Math.abs(m_frontMotor.getEncoder().getVelocity()) >= 6000
        && Math.abs(m_backMotor.getEncoder().getVelocity()) >= 6000;
  }

  // Function to clamp the power to a value between -1 and 1
  public static double clampPower(double power) {
    return MathUtil.clamp(power, -1, 1);
  }
}
