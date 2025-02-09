package frc.robot.subsystems.coral;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;

public class CoralStateManager {
    private static CoralState m_currentState = CoralStates.kRest;


    public CoralState getState() {
        return m_currentState;
    }

    public void setState(CoralState coralState) {
        m_currentState = coralState;
        update();
    }

    public void update() {
        subsystems.elevator.setState(m_currentState);
        subsystems.coralArm.setState(m_currentState);
        subsystems.coralIntake.setState(m_currentState);
    }
}
