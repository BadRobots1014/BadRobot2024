package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final CANSparkMax m_flippyMotor;
  public final CANSparkMax m_intakeMotor;
  public final RelativeEncoder m_flippyEncoder;

  public final SparkLimitSwitch m_intakeLimitSwitch;

  public final ShuffleboardTab m_tab;

  public IntakeSubsystem() {

    //Intake mechanism motors
    m_flippyMotor = new CANSparkMax(IntakeConstants.kFlippyCanId, MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(IntakeConstants.kGroundIntakeCanId, MotorType.kBrushed);
    m_flippyEncoder = m_flippyMotor.getEncoder(Type.kHallSensor, 42);

    m_flippyMotor.setIdleMode(IdleMode.kCoast);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_flippyMotor.setInverted(false);
    m_intakeMotor.setInverted(false);

    m_intakeLimitSwitch = m_intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_tab = Shuffleboard.getTab("Intake");
    m_tab.addNumber("Intake Current", this::getIntakeCurrent);
    m_tab.addNumber("Flipper Position", this::getFlipperEncoder);
    m_tab.addNumber("Flipper Current", this::getFlipperAmps);

    resetFlipperEncoder();
  }

  //Intake
  public void intake(double power) {m_intakeMotor.set(-power);}
  public void stopIntake() {m_intakeMotor.stopMotor();}
  public double getIntakeCurrent() {return m_intakeMotor.getOutputCurrent();}
  public boolean intakeCurrentSpike() {return getIntakeCurrent() >= IntakeConstants.kIntakeMaxCurrent;}
  public void intakeCurrentSensitive(double power) {
    if (!intakeCurrentSpike()) intake(power);
    else stopIntake();
  }

  //Flipper
  public void flip(double power) {m_flippyMotor.set(-power);}
  public void stopFlipper() {m_flippyMotor.stopMotor();}
  public void flipToPosition(double pos) {
    // Moves to the goal encoder angle
    flip(MathUtil.clamp(pos - m_flippyEncoder.getPosition(), -1, 1));
  }
  public double getFlipperEncoder() {return -m_flippyEncoder.getPosition() / IntakeConstants.kFlipperGearRatio;}
  public void resetFlipperEncoder() {m_flippyEncoder.setPosition(0);}
  public double getFlipperAmps(){return m_flippyMotor.getOutputCurrent();}

  //Combo
  public void intakeFromGround() {
    if (getFlipperEncoder() < .3) {
      flip(.3);
      stopIntake();
    }
    else {
      intake(1);
      stopFlipper();
    }
  }
  public void dropFlipper(){
    if (getFlipperEncoder() < .3) {
      flip(.3);
      stopIntake();
    }
    else {
      //intake(1);
      stopFlipper();
    }
  }
  public void retractIntake() {
    if (getFlipperEncoder() > .1) flip(-.3);
    else stopFlipper();
  }
  public void expelRing() {
    intake(-1);
  }
  public void expelRingGround() {
    if (getFlipperEncoder() < .3) {
      flip(.3);
      stopIntake();
    }
    else {
      intake(-1);
      stopFlipper();
    }
  }
  public void feedShooter() {
    retractIntake();
    if (getFlipperEncoder() < .1) expelRing();
    else stopFlipper();
  }
  public void stopEverything() {
    stopFlipper();
    stopIntake();
  }
  

}
