package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveDriveDistanceCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShootPath extends SequentialCommandGroup {
    public AutoShootPath(SwerveSubsystem swerve, ShooterSubsystem shoot){
        //Auto code based off of time
        super (
            new SwerveDriveCommand(swerve, supplyDouble(0), supplyDouble(0.3),supplyDouble(-0.3),supplyBoolean(false),supplyBoolean(false))
            .withTimeout(3),
            new ShootCommand(shoot).withTimeout(4)
        );

        /* //Auto code based of of Nathan's auto commands which don't work at the time of writing
        super (
            new SwerveDriveDistanceCommand(swerve, inchesToMeters(150), 0),
            new SwerveDriveTurnThetaCommand(swerve, 45),
            new ShootCommand(shoot).withTimeout(4)
        );
        */
    }

    private static Supplier<Double> supplyDouble(double d) {return new Supplier<Double>() {@Override public Double get() {return d;}};}
    private static Supplier<Boolean> supplyBoolean(boolean b) {return new Supplier<Boolean>() {@Override public Boolean get() {return b;}};}
    private static double inchesToMeters(double inches){
        return inches * 0.0254;
    }
}
