// POSITIVE COUNTERCLOCKWISE
// NEGATIVE CLOCKWISE
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Shooter");

  private final GenericEntry m_frontMotorPower;
  private final GenericEntry m_backMotorPower;
  private final GenericEntry m_frontIntakePower;
  private final GenericEntry m_backIntakePower;
  private final GenericEntry m_indexPower;
  private final GenericEntry m_winchUpPower;
  private final GenericEntry m_winchDownPower;

  public final CANSparkFlex m_frontMotor;
  public final CANSparkFlex m_backMotor;
  public final CANSparkMax m_indexMotor;
  public final CANSparkMax m_winchMotor;
  public final RelativeEncoder m_winchEncoder;

  public ShooterSubsystem() {

    m_frontMotor = new CANSparkFlex(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_backMotor = new CANSparkFlex(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
    m_indexMotor = new CANSparkMax(ShooterConstants.kIndexMotorCanId, MotorType.kBrushless);
    m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorCanId, MotorType.kBrushed);
    m_frontMotor.setIdleMode(IdleMode.kCoast);
    m_backMotor.setIdleMode(IdleMode.kCoast);
    m_indexMotor.setIdleMode(IdleMode.kCoast);
    m_winchMotor.setIdleMode(IdleMode.kCoast);
    m_winchEncoder = m_winchMotor.getEncoder(Type.kQuadrature, 8192);

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
    m_indexPower = m_shuffleboardtab.add("Index Power", ShooterConstants.kIndexPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_winchUpPower = m_shuffleboardtab.add("Winch Up Power", ShooterConstants.kWinchUpPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_winchDownPower = m_shuffleboardtab.add("Winch Down Power", ShooterConstants.kWinchDownPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();

    // Winch encoder value
    m_shuffleboardtab.addNumber("Winch Encoder", this::getWinchEncoder);
  }

  // Shooter stuff
  public double[] getShooterPowers() { // Function to get the motor powers from shuffleboard and clamp them to a value between -1 and 1
    return new double[] {
      clampPower(m_frontMotorPower.getDouble(ShooterConstants.kFrontShootPower)),
      clampPower(m_backMotorPower.getDouble(ShooterConstants.kBackShootPower))
    };
  }
  public double[] getIntakePowers() {
    return new double[] {
      clampPower(m_frontIntakePower.getDouble(ShooterConstants.kFrontIntakePower)),
      clampPower(m_backIntakePower.getDouble(ShooterConstants.kBackIntakePower))
    };
  }
  public void runShooter() {
    m_frontMotor.set(getShooterPowers()[0]);
    m_backMotor.set(getShooterPowers()[1]);
  }
  public void runShooter(double rearPower) {
    m_frontMotor.set(getShooterPowers()[0]);
    m_backMotor.set(rearPower);
  }
  public void runIntake() {
    m_frontMotor.set(getIntakePowers()[0]);
    m_backMotor.set(getIntakePowers()[1]);
  }
  public void stopShooter() {
    m_frontMotor.stopMotor();
    m_backMotor.stopMotor();
  }
  public boolean isShooterRunning() {
    if (m_frontMotor.get() == 0 && m_backMotor.get() == 0) { //Check if both motors are stopped
      return false; //If so, the shooter is not running
    } else {
      return true; //Otherwise, the shooter is running
    }
  }

  // Indexer stuff
  public double getIndexPower() {return clampPower(m_indexPower.getDouble(ShooterConstants.kIndexPower));}
  public void runIndex() {m_indexMotor.set(getIndexPower());}
  public void stopIndex() {m_indexMotor.stopMotor();}

  // Winch stuff
  public double[] getWinchPowers() {
    return new double[] {
      clampPower(m_winchUpPower.getDouble(ShooterConstants.kWinchUpPower)),
      clampPower(m_winchDownPower.getDouble(ShooterConstants.kWinchDownPower))
    };
  }
  public void winchUp() {m_winchMotor.set(getWinchPowers()[0]);}
  public void winchDown() {m_winchMotor.set(getWinchPowers()[1]);}
  public void stopWinch() {m_winchMotor.stopMotor();}
  public void manualWinch(double power) {m_winchMotor.set(power);}
  public double getWinchEncoder() {return m_winchEncoder.getPosition();}
  public void resetWinchEncoder() {m_winchEncoder.setPosition(0);}

  // Function to clamp the power to a value between -1 and 1
  public static double clampPower(double power) {
    return MathUtil.clamp(power, -1, 1);
  }
}
