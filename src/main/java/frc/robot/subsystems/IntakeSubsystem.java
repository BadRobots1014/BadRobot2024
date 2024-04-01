package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final CANSparkMax m_flippyMotor;
  public final CANSparkMax m_intakeMotor;
  public final RelativeEncoder m_flippyEncoder;

  public IntakeSubsystem() {

    //Intake mechanism motors
    m_flippyMotor = new CANSparkMax(IntakeConstants.kFlippyCanId, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(IntakeConstants.kGroundIntakeCanId, MotorType.kBrushed);
    m_flippyEncoder = m_flippyMotor.getEncoder(Type.kHallSensor, 42);

    m_flippyMotor.setIdleMode(IdleMode.kBrake);
    m_intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  //Intake
  public void intake(double power) {m_intakeMotor.set(power);}
  public void stopIntake() {m_intakeMotor.stopMotor();}
  public double getIntakeCurrent() {return m_intakeMotor.getOutputCurrent();}
  public boolean intakeCurrentSpike() {return getIntakeCurrent() >= IntakeConstants.kIntakeMaxCurrent;}
  public void intakeCurrentSensitive(double power) {
    if (!intakeCurrentSpike()) intake(power);
    else stopIntake();
  }

  //Flipper
  public void flip(double power) {m_flippyMotor.set(power);}
  public void stopFlipper() {m_flippyMotor.stopMotor();}
  public void flipToPosition(double pos) {
    // Moves to the goal encoder angle
    flip(MathUtil.clamp(pos - m_flippyEncoder.getPosition(), -1, 1));
  }

  //Combo
  public void intakeFromGround() {
    if (m_flippyEncoder.getPosition() < .3 /* To be changed */) flip(.5);
    else intakeCurrentSensitive(1);
  }
  public void retractIntake() {
    if (m_flippyEncoder.getPosition() > .1) flip(-.5);
  }
  public void expelRing() {
    intake(-1);
  }
  public void feedShooter() {
    retractIntake();
    expelRing();
  }

}
