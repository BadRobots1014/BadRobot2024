package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
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

    private final GenericEntry m_frontMotorPower;
    private final GenericEntry m_backMotorPower;
    //private final NetworkTable m_tab = NetworkTableInstance.getDefault().getTable("Shooter");
    public final CANSparkMax m_frontMotor;
    public final CANSparkMax m_backMotor;
    private
     static boolean ShooterRunning = false;
    
    public ShooterSubsystem(double defaultfront, double defaultback) {
        m_frontMotorPower = m_shuffleboardtab.add("Front Motor Power",defaultfront).withWidget(BuiltInWidgets.kNumberSlider)
                                                                              .withProperties(Map.of("min",-1.0,"max",1.0))
                                                                              .getEntry();

        m_backMotorPower = m_shuffleboardtab.add("Back Motor Power",defaultback).withWidget(BuiltInWidgets.kNumberSlider)
                                                                              .withProperties(Map.of("min",-1.0,"max",1.0))
                                                                              .getEntry();
                                                                              
        m_shuffleboardtab.addBoolean("Motor Spinning", () -> ShooterSubsystem.IsShooterRunning());

        m_frontMotor = new CANSparkMax(ShooterConstants.kFrontMotor, MotorType.kBrushed); // Assuming brushless? 
        m_backMotor = new CANSparkMax(ShooterConstants.kBackMotor, MotorType.kBrushed);
    }

    private static double clampPower(double power) {
        return MathUtil.clamp(power, -1.0, 1.0);
    }
    // power of moter is in the range from -1.0 to 1.0

    public double[] GetPower() {
        return new double[] {m_frontMotorPower.getDouble(0.0), m_backMotorPower.getDouble(0.0)};
    }

    public void SetFrontMotorPower(double fracpower) {
        m_frontMotorPower.setDouble(fracpower);
        //m_motorpower = fracpower;
    }

    public void SetBackMotorPower(double fracpower) {
      m_backMotorPower.setDouble(fracpower);
    }

    public void  runShooter() {
        m_frontMotor.set(clampPower(m_frontMotorPower.getDouble(0.0)));
        m_backMotor.set(clampPower(m_backMotorPower.getDouble(0.0)));

        System.out.print("Run shooter function code invoked\n");

        ShooterRunning = true;
    }

    public void stopShooter() {
        m_frontMotor.stopMotor();
        m_backMotor.stopMotor();

        System.out.print("stop shooter function code invoked\n");

        ShooterRunning = false;
    }

    public void ShootCycle() {
        if (!ShooterRunning) {
            System.out.print("WARNING: ShooterSubsystem tried a shoot cycle when the motors were not running!");
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