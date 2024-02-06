package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;

public class ShooterSubsystem extends SubsystemBase {
  private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Shooter");

  private final GenericEntry m_frontMotorPower;
  private final GenericEntry m_backMotorPower;

  public final CANSparkMax m_frontMotor;
  public final CANSparkMax m_backMotor;
  private static boolean shooterRunning = false;

  public ShooterSubsystem(double defaultpower) {
    m_frontMotorPower = m_shuffleboardtab.add("Front Motor Power", defaultpower)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    m_backMotorPower = m_shuffleboardtab.add("Back Motor Power", defaultpower).withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -1.0, "max", 1.0))
        .getEntry();

    m_shuffleboardtab.addBoolean("Motor Spinning", () -> ShooterSubsystem.isShooterRunning());

    m_frontMotor = new CANSparkMax(ShooterConstants.kFrontMotorCanId, MotorType.kBrushless);
    m_backMotor = new CANSparkMax(ShooterConstants.kBackMotorCanId, MotorType.kBrushless);
  }

  private static double clampPower(double power) {
    return MathUtil.clamp(power, -1.0, 1.0);
  }

  public double[] getPower() {
    return new double[] { m_frontMotorPower.getDouble(0.0), m_backMotorPower.getDouble(0.0) };
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
    double[] powers = getPower();
    m_frontMotor.set(clampPower(powers[0]));
    m_backMotor.set(clampPower(powers[1]));
    System.out.println("Run shooter function code invoked\n");
    shooterRunning = true;
  }

  public void stopShooter() {
    m_frontMotor.stopMotor();
    m_backMotor.stopMotor();
    System.out.print("stop shooter function code invoked\n");
    shooterRunning = false;
  }

  public void shootCycle() {
    if (!shooterRunning) {
      System.out.print("WARNING: ShooterSubsystem tried a shoot cycle when the motors were not running!\n");
      return;
    }
    /* code to shoot a single ring goes here */
  }

  public static boolean isShooterRunning() {
    return shooterRunning;
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