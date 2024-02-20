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
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {

  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab(
    "Shooter"
  );

  private final GenericEntry m_frontMotorPower;
  private final GenericEntry m_backMotorPower;
  private final GenericEntry m_intakePower;
  public final CANSparkFlex m_frontMotor;
  public final CANSparkFlex m_backMotor;

  public ShooterSubsystem(double defaultShooterPower, double defaultIntakePower) {

    m_frontMotor =
      new CANSparkFlex(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_frontMotor.setIdleMode(IdleMode.kBrake);
    m_backMotor =
      new CANSparkFlex(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
    m_backMotor.setIdleMode(IdleMode.kBrake);
    // Displays whether or not the shooter is running
    m_shuffleboardtab.addBoolean("Motor Spinning", this::isShooterRunning);

    // Shuffleboard number slider for front motor power
    m_frontMotorPower =
      m_shuffleboardtab
        .add("Front Motor Power", defaultShooterPower)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    // Shuffleboard number slider for the back motor power
    m_backMotorPower =
      m_shuffleboardtab
        .add("Back Motor Power", defaultShooterPower)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();
    
    m_intakePower = m_shuffleboardtab
    .add("Intake Power", defaultIntakePower)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1.0, "max", 1.0))
    .getEntry();
  }

  // Function to run the shooter motors
  public void runShooter() {
    double[] powers = getShooterPower();
    m_frontMotor.set(powers[0]);
    m_backMotor.set(powers[1]);
  }

  public void runShooter(double rearPower) {
    double [] powers = getShooterPower();
    m_frontMotor.set(powers[0]);
    m_backMotor.set(rearPower);
  }

  public void runIntake() {
    double [] powers = getIntakePower();
    m_frontMotor.set(powers[0]);
    m_backMotor.set(powers[1]);
  }

  // Function to stop the shooter motors
  public void stopShooter() {
    m_frontMotor.stopMotor();
    m_backMotor.stopMotor();
  }

  // Function to clamp the power to a value between -1 and 1
  public static double clampPower(double power) {
    return MathUtil.clamp(power, -1, 1);
  }

  // Function to get the motor powers from shuffleboard and clamp them to a value
  // between -1 and 1
  public double[] getShooterPower() {
    return new double[] {
      clampPower(m_frontMotorPower.getDouble(0.0)),
      clampPower(m_backMotorPower.getDouble(0.0)),
    };
  }

  public double [] getIntakePower() {
    return new double [] {
      clampPower(m_intakePower.getDouble(-0.35)),
      0.0,
    };
  }

  // Function to check if the shooter is running
  // I know this looks like it's inverted, but for some reason it works in
  // shuffleboard so dont change it
  public boolean isShooterRunning() {
    if (m_frontMotor.get() == 0 && m_backMotor.get() == 0) {
      return false;
    } else {
      return true;
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
