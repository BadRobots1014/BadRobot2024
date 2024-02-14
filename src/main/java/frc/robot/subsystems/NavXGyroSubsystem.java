package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXGyroSubsystem extends SubsystemBase {

  private final AHRS navx;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Navx");

  //Gyro offsets in the x, y, and z directions
  private double xOffset, yOffset, zOffset;

  public NavXGyroSubsystem() {
    navx = new AHRS(SPI.Port.kMXP);
    xOffset = 0;
    yOffset = 0;
    zOffset = 0;
    m_tab.addNumber("Yaw", this::getYaw);
    m_tab.addNumber("Roll", this::getRoll);
    m_tab.addNumber("Pitch", this::getPitch);
    m_tab.addNumber("X Displacement", this::getDisplacementX);
    m_tab.addNumber("Y Displacement", this::getDisplacementY);
    m_tab.addNumber("Z Displacement", this::getDisplacementZ);
  }

  public void periodic() {}

  public double getYaw() {return navx.getYaw();}
  public double getPitch() {return navx.getPitch();}
  public double getRoll() {return navx.getRoll();}
  public double getAngle() {return navx.getAngle();}

  public double getDisplacementX() {return navx.getDisplacementX() + xOffset;}
  public double getDisplacementY() {return navx.getDisplacementY() + yOffset;}
  public double getDisplacementZ() {return navx.getDisplacementZ() + zOffset;}

  public double getVelocityX() {return navx.getVelocityX();}
  public double getVelocityY() {return navx.getVelocityY();}
  public double getVelocityZ() {return navx.getVelocityZ();}

  public double getRate() {return navx.getRate();}

  public void reset() {
    navx.reset();
    navx.resetDisplacement();
  }

  public void setPose(Pose2d pose) {
    reset();
    xOffset = pose.getX();
    yOffset = pose.getY();
    navx.setAngleAdjustment(pose.getRotation().getDegrees());
  }
}
