package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class SetLEDRed extends Command {

    private final LED m_led;
    private int count = 0;

    public SetLEDRed(LED m_led) {
        this.m_led = m_led;

        addRequirements(m_led);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        for (int i = 0; i < m_led.getLength(); i++) {
            m_led.setLED(i, 255, 0, 0);
        }
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (count < m_led.getLength()) {
            m_led.setLED(count, 255 - (255 / m_led.getLength() * count), (255 / m_led.getLength()) * count, 0);
            count++;
        }
        else {
            count = 0;
            m_led.setLED(count, 255, 0, 0);
        }

    }

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
