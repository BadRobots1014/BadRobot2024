// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveSubsystem;

// /** An example command that uses an example subsystem. */
// public class TurnPresetCycle extends Command {

//   @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//   private final SwerveSubsystem m_subsystem;
//   private final double[] angles = new double[] {0, 90, 180, 270};
//   private int currentIndex = 0;

//   private TurnToThetaCommand thetaCommand;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public TurnPresetCycle(SwerveSubsystem subsystem) {
//     m_subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//     thetaCommand = new TurnToThetaCommand(m_subsystem, angles[0]);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() 
//   {
//     thetaCommand.execute();
//     if (currentIndex == angles.length-1)
//         currentIndex = -1;
//     thetaCommand.setTargetTheta(angles[currentIndex+1]);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
