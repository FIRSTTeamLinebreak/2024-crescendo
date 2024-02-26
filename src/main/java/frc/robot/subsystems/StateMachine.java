package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase {

    public static enum State {
        stow,
        amp,
        speaker,
        trap,
        climb,
        intake
    }

    public static String stateToString(State currentState) {
        switch (currentState) {
            case stow:
                return "Stow";
            case amp:
                return "Amp";
            case speaker:
                return "Speaker";
            case trap:
                return "Trap";
            case climb:
                return "Climb";
            case intake:
                return "Intake";
        }
        return "";
    }

    private final Launcher m_launcher;
    private final Elevator m_elevator;
    private final Vision m_vision;

    private State currentState;
    private State lastState;

    public StateMachine(Launcher m_launcher, Elevator m_elevator, Vision m_vision) {
        this.m_launcher = m_launcher;
        this.m_elevator = m_elevator;
        this.m_vision = m_vision;
    }

    public State getState() {
        return currentState;
    }   
    
    public void setState(State state) {
        this.lastState = currentState;
        this.currentState = state;
    }



    @Override
    public void periodic() {

    }
    
}
