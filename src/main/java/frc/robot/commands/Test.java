package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Test extends Command {

    private final Launcher launcher;
    private final boolean intake;

    public Test(Launcher launcher, boolean intake) {
        this.launcher = launcher;
        this.intake = intake;

        addRequirements(launcher);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        if(intake) {
            launcher.toggleIntake();
        }
        else {
            launcher.toggleLauncher();
        }
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {}

    /**
     * Called when either the command finishes normally, or when it interrupted/canceled. Do not
     * schedule commands here that share requirements with this command. Use andThen(Command)
     * instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {}

    /**
     * Whether the command has finished. If true, calls end() and stops the command from executing
     *
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
