package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem {
    //private double m_motorpower = 0.0; <-- not sure we even need this var...

    private final ShuffleboardTab m_shuffleboardtab = Shuffleboard.getTab("Shooter");
    private final GenericEntry m_speedentry;
    //private final NetworkTable m_tab = NetworkTableInstance.getDefault().getTable("Shooter");

    // power of moter is in the range from 0.0 to 0.1
    public ShooterSubsystem(double defaultpower) {
        m_speedentry = m_shuffleboardtab.add("Motor Speed",defaultpower).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).getEntry();

    }

    public double GetPower() {
        return m_speedentry.getDouble(-1);
    }

    public void SetPower(double fracpower) {
        m_speedentry.setDouble(fracpower);
        //m_motorpower = fracpower;
    }
}