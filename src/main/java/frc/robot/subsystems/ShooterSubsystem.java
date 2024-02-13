package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
/*
  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab(
    "Shooter"
  );

  private final GenericEntry m_frontMotorPower;
  private final GenericEntry m_backMotorPower;
*/
  public final CANSparkFlex m_frontMotor;
  public final CANSparkFlex m_backMotor;

  public ShooterSubsystem(double defaultpower) {
    /*
    m_frontMotorPower =
      m_shuffleboardtab
        .add("Front Motor Power", defaultpower)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    m_backMotorPower =
      m_shuffleboardtab
        .add("Back Motor Power", defaultpower)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    m_shuffleboardtab.addBoolean(
      "Motor Spinning",
      () -> ShooterSubsystem.isShooterRunning()
    );
*/
    m_frontMotor =
      new CANSparkFlex(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_backMotor =
      new CANSparkFlex(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
  }

  /*
   * It appears that these methods are not needed?
   * If they are then feel free to uncomment.
   */

  /*
   * public void setFrontMotorPower(double fracpower) {
   * m_frontMotorPower.setDouble(fracpower);
   * }
   *
   * public void setBackMotorPower(double fracpower) {
   * m_backMotorPower.setDouble(fracpower);
   * }
   */

  public void runShooter() {
    m_frontMotor.set(0.5);
    m_backMotor.set(0.5);
    System.out.println("Run shooter function code invoked\n");
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
