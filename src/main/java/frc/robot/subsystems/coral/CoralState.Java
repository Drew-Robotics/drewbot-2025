package frc.robot.subsystems.coral;

public class CoralState {

    private final Distance m_elevatorSetpoint;
    private final Rotation2d m_armSetpoint;
    private boolean m_intakeRunning;

    CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint) {
        CoralState(elevatorSetpoint, armSetpoint, false);
    }

    CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint, boolean intakeRunning) {
        m_elevatorSetpoint = elevatorSetpoint;
        m_armSetpoint = armSetpoint;
        m_intakeRunning = intakeRunning
    }
}