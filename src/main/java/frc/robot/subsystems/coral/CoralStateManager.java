package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public CoralState getState() {
        return m_currentState;
    }

    public void setState(CoralState coralState) {
        m_currentState = coralState;
        update();
    }

    private void update() {
        subsystems.elevator.setState(m_currentState);
        subsystems.coralArm.setState(m_currentState);
        subsystems.coralIntake.setState(m_currentState);
    }

    public Command getSetStateCommand(CoralState coralState) {
        return new InstantCommand(
            () -> setState(coralState), 
            this, subsystems.coralArm, subsystems.elevator
        );
    }

    /* Overrides */
    protected void dashboardPeriodic() {}

    protected void dashboardInit() {
        SmartDashboard.putString("Coral State", getState().toString());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
