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
import frc.robot.Constants.WinchySquinchyConstants;

import java.util.Map;

public class WinchySubsystem extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Winchy");

  private final GenericEntry m_winchUpPower, m_winchDownPower;

  public final CANSparkMax m_winchMotor;
  public final RelativeEncoder m_winchEncoder;

  public WinchySubsystem() {
    m_winchMotor = new CANSparkMax(ShooterConstants.kWinchMotorCanId, MotorType.kBrushed);
    m_winchMotor.setIdleMode(IdleMode.kBrake);
    m_winchMotor.setInverted(false);
    m_winchEncoder = m_winchMotor.getEncoder(Type.kQuadrature, 8192);

    // Shuffleboard numbers for the winchy squinchy
    m_winchUpPower = m_shuffleboardtab.add("Winch Up Power", WinchySquinchyConstants.kWinchUpPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();
    m_winchDownPower = m_shuffleboardtab.add("Winch Down Power", WinchySquinchyConstants.kWinchDownPower)
      .withProperties(Map.of("min", -1.0, "max", 1.0)).getEntry();

    // Winch encoder value
    m_shuffleboardtab.addNumber("Winch Encoder", this::getWinchEncoder);
  }

  // Winch stuff
  public double[] getWinchPowers() {
    return new double[] {
      clampPower(m_winchUpPower.getDouble(WinchySquinchyConstants.kWinchUpPower)),
      clampPower(m_winchDownPower.getDouble(WinchySquinchyConstants.kWinchDownPower))
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
