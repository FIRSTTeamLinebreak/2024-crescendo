package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.visionLauncherRotation;

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
    private final Intake m_intake;

    private State currentState;
    private State lastState;

    public StateMachine(Launcher m_launcher, Elevator m_elevator, Vision m_vision, Intake m_intake) {
        this.m_launcher = m_launcher;
        this.m_elevator = m_elevator;
        this.m_vision = m_vision;
        this.m_intake = m_intake;
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
        if (currentState != lastState) {
            new InstantCommand(CommandScheduler.getInstance()::cancelAll);
            switch (currentState) {
                case stow:
                    switch (lastState) {
                        case stow:
                            return;
                        case amp:
                        case intake:
                            m_elevator.moveToSetpoint(50)
                                    .alongWith(m_launcher.moveClawToSetpoint(0.5))
                                    .andThen(m_launcher.moveClawToSetpoint(1.0)
                                            .alongWith(m_elevator.moveToSetpoint(10)))
                                    .schedule();
                            break;
                        case speaker:
                        case trap:
                            m_elevator.moveToSetpoint(10)
                                        .alongWith(m_launcher.moveClawToSetpoint(1.0))
                                    .schedule();
                            break;
                        case climb: // @TODO
                    }
                    break;
                case amp:
                    switch (lastState) {
                        case stow:
                            m_launcher.moveClawToSetpoint(0.50)
                                    .alongWith(m_elevator.moveToSetpoint(50))
                                    .andThen(m_elevator.moveToSetpoint(100)
                                            .alongWith(m_launcher.moveClawToSetpoint(0.27)))
                                    .schedule();
                            break;
                        case intake:
                            m_elevator.moveToSetpoint(100)
                                    .alongWith(m_launcher.moveClawToSetpoint(0.27))
                                    .schedule();
                                    break;
                        default:
                    }
                    break;
                case speaker:
                    switch (lastState) {
                        case stow:
                            new visionLauncherRotation(m_launcher, m_vision, m_elevator).schedule();;
                            break;
                        case intake:
                            m_launcher.moveClawToSetpoint(0.50)
                                    .alongWith(m_elevator.moveToSetpoint(50))
                                    .andThen(m_elevator.moveToSetpoint(100)
                                            .alongWith(new visionLauncherRotation(m_launcher, m_vision, m_elevator)))
                                    .schedule();
                                break;
                        default:
                    }
                    break;
                case trap:
                    switch (lastState) {
                        case stow:
                            m_launcher.moveClawToSetpoint(.905)
                                    .alongWith(m_elevator.moveToSetpoint(48))
                                .schedule();
                                break;
                        case intake:
                            m_elevator.moveToSetpoint(50)
                                .alongWith(m_launcher.moveClawToSetpoint(0.5))
                                .andThen(m_launcher.moveClawToSetpoint(.905)
                                    .alongWith(m_elevator.moveToSetpoint(48)))
                                .schedule();
                                break;
                    }
                    break;
                case climb:
                    switch (lastState) {
                        case stow:
                        case trap:
                            m_elevator.moveToSetpoint(50)
                                    .alongWith(m_launcher.moveClawToSetpoint(.5))
                                .andThen(m_elevator.moveToSetpoint(100)
                                    .alongWith(m_launcher.moveClawToSetpoint(.1)))
                            .schedule();
                            break;
                    }
                case intake:
                    switch (lastState) {
                        case stow:
                            m_launcher.setControlSpeed(-.05);
                            m_elevator.moveToSetpoint(50)
                                .alongWith(m_launcher.moveClawToSetpoint(.5)
                            .andThen(m_launcher.moveClawToSetpoint(1.0))
                                .alongWith(m_elevator.moveToSetpoint(10)))
                            .schedule();
                            break;
                        case intake:
                            m_elevator.moveToSetpoint(50)
                                .alongWith(m_launcher.moveClawToSetpoint(0.5))
                            .andThen(m_launcher.moveClawToSetpoint(0.042))
                            .andThen(m_elevator.moveToSetpoint(37)).schedule();
                            break;
                        

                    }
                    break;
            }
        }
        
        else {
            switch (currentState) {
                case intake:
                        m_launcher.setLauncherSpeed(0.3);
                        m_launcher.setControlSpeed(0.1);
                        m_intake.setSpeed(0.3);
                    break;
                default:
                    break;
            }
        }
    }

    public Command Stow() {
        return new InstantCommand(() -> this.setState(State.stow));
    }

    public Command Speaker() {
        return new InstantCommand(() -> this.setState(State.speaker));
    }

    public Command Trap() {
        return new InstantCommand(() -> this.setState(State.trap));
    }

    public Command Intake() {
        return new InstantCommand(() -> this.setState(State.intake));
    }

    public Command Climb() {
        return new InstantCommand(() -> this.setState(State.climb));
    }

    public Command Amp() {
        return new InstantCommand(() -> this.setState(State.amp));
    }

}
