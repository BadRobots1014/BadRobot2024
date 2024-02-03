package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

// Used to only run a function on a certain trigger
// to avoid boilerplate of creating another command class just to stick a function call in it's execute
public class RunFunction extends Command {
    private final Runnable m_func;

    RunFunction(Runnable func) {
        m_func = func;
    }

    @Override
    public void execute() {
        m_func.run();
    }
}
