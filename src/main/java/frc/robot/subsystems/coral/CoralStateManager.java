package frc.robot.subsystems.coral;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.subsystems.SubsystemAbstract;

public class CoralStateManager extends SubsystemAbstract{
    private static CoralState m_currentState = CoralStates.kRest;


    protected static CoralStateManager m_instance;
    public static CoralStateManager getInstance() {
        if (m_instance == null)
            m_instance = new CoralStateManager();
        return m_instance;
    }

    public CoralStateManager() {
        super();
    }

    protected void dashboardPeriodic() {}
    protected void dashboardInit() {}
    protected void publishInit() {}
    protected void publishPeriodic() {}

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
