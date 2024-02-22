// POSITIVE COUNTERCLOCKWISE
// NEGATIVE CLOCKWISE
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab(
    "Shooter"
  );

  private final GenericEntry m_frontMotorPower;
  private final GenericEntry m_backMotorPower;
  private final GenericEntry m_frontIntakePower;
  private final GenericEntry m_backIntakePower;
  private final GenericEntry m_indexPower;

  public final CANSparkFlex m_frontMotor;
  public final CANSparkFlex m_backMotor;
  public final CANSparkFlex m_indexMotor;

  public ShooterSubsystem() {

    m_frontMotor = new CANSparkFlex(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_backMotor = new CANSparkFlex(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
    m_indexMotor = new CANSparkFlex(ShooterConstants.kIndexMotorCanId, MotorType.kBrushless);
    m_frontMotor.setIdleMode(IdleMode.kBrake);
    m_backMotor.setIdleMode(IdleMode.kBrake);
    m_indexMotor.setIdleMode(IdleMode.kBrake);
    // Displays whether or not the shooter is running
    m_shuffleboardtab.addBoolean("Motor Spinning", this::isShooterRunning);

    // Shuffleboard number slider for front motor power
    m_frontMotorPower = m_shuffleboardtab
      .add("Front Motor Power", ShooterConstants.kFrontShootPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .getEntry();

    // Shuffleboard number slider for the back motor power
    m_backMotorPower = m_shuffleboardtab
      .add("Back Motor Power", ShooterConstants.kBackShootPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .getEntry();
    
    m_frontIntakePower = m_shuffleboardtab
      .add("Intake Power", ShooterConstants.kFrontIntakePower)
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .getEntry();

    m_backIntakePower = m_shuffleboardtab
      .add("Intake Power", ShooterConstants.kBackIntakePower)
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .getEntry();

    m_indexPower = m_shuffleboardtab
      .add("Index Power", ShooterConstants.kIndexPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0))
      .getEntry();
  }

  // Function to run the shooter motors
  public void runShooter() {
    m_frontMotor.set(getShooterPower()[0]);
    m_backMotor.set(getShooterPower()[1]);
  }

  public void runShooter(double rearPower) {
    m_frontMotor.set(getShooterPower()[0]);
    m_backMotor.set(rearPower);
  }

  public void runIntake() {
    m_frontMotor.set(getIntakePower()[0]);
    m_backMotor.set(getIntakePower()[1]);
  }

  // Function to stop the shooter motors
  public void stopShooter() {
    m_frontMotor.stopMotor();
    m_backMotor.stopMotor();
  }

  // Indexer stuff
  public void runIndex() {
    m_indexMotor.set(getIndexPower());
  }

  public void stopIndex() {
    m_indexMotor.stopMotor();
  }

  // Function to clamp the power to a value between -1 and 1
  public static double clampPower(double power) {
    return MathUtil.clamp(power, -1, 1);
  }

  // Function to get the motor powers from shuffleboard and clamp them to a value between -1 and 1
  public double[] getShooterPower() {
    return new double[] {
      clampPower(m_frontMotorPower.getDouble(ShooterConstants.kFrontShootPower)),
      clampPower(m_backMotorPower.getDouble(ShooterConstants.kBackShootPower)),
    };
  }

  public double[] getIntakePower() {
    return new double[] {
      clampPower(m_frontIntakePower.getDouble(ShooterConstants.kFrontIntakePower)),
      clampPower(m_backIntakePower.getDouble(ShooterConstants.kBackIntakePower)),
    };
  }

  public double getIndexPower() {
    return m_indexPower.getDouble(ShooterConstants.kIndexPower);
  }

  /* 
   * In response to the comment below, it works because it checks if both motors are set to zero and
   * returns that the shooter is not running if that's the case. Otherwise it returns that the shooter
   * is indeed running because at least one motor is not at zero power.
   */
  // Function to check if the shooter is running
  // I know this looks like it's inverted, but for some reason it works in shuffleboard so dont change it
  public boolean isShooterRunning() {
    if (m_frontMotor.get() == 0 && m_backMotor.get() == 0) {
      return false;
    } else {
      return true;
    }
  }
}
