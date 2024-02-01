package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;



/* keeping these incase we need them
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
*/

public class ShooterSubsystem extends SubsystemBase{
    //private double m_motorpower = 0.0; <-- not sure we even need this var...

    private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Shooter");
    private final GenericEntry m_speedentry;
    //private final NetworkTable m_tab = NetworkTableInstance.getDefault().getTable("Shooter");
    public final CANSparkMax m_leftMotor;
    public final CANSparkMax m_rightMotor;
    private
     static boolean ShooterRunning = false;
    
    public ShooterSubsystem(double defaultpower) {
        m_speedentry = m_shuffleboardtab.add("Motor Power",defaultpower).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",-1.0,"max",1.0)).getEntry();
        m_leftMotor = new CANSparkMax(ShooterConstants.kRightDeviceId, MotorType.kBrushless); // Assuming brushless? 
        m_rightMotor = new CANSparkMax(ShooterConstants.kLeftDeviceId, MotorType.kBrushless);
    }

    private static double clampPower(double power) {
        return MathUtil.clamp(power, -1.0, 1.0);
    }
    // power of moter is in the range from -1.0 to 1.0

    public double GetPower() {
        return m_speedentry.getDouble(0.0);
    }

    public void SetPower(double fracpower) {
        m_speedentry.setDouble(fracpower);
        //m_motorpower = fracpower;
    }

    public void runShooter() {
        m_leftMotor.set(clampPower(m_speedentry.getDouble(0.0)));
        m_rightMotor.set(clampPower(m_speedentry.getDouble(0.0)));

        ShooterRunning = false;
    }

    public void stopShooter() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();

        ShooterRunning = false;
    }

    public void ShootCycle() {
        if (!ShooterRunning) {
            System.console().printf("WARNING: ShooterSubsystem tried a shoot cycle when the motors were not running!");
            return;
        }
        /*code to shoot a single ring goes here*/
    }

    public static boolean IsShooterRunning() {
        return ShooterRunning;
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