package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class CoralState {
    private final Distance m_elevatorSetpoint;
    private final Rotation2d m_armSetpoint;
    private boolean m_intakeRunning;

    public CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint) {
        this(elevatorSetpoint, armSetpoint, false);
    }

    public CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint, boolean intakeRunning) {
        m_elevatorSetpoint = elevatorSetpoint;
        m_armSetpoint = armSetpoint;
        m_intakeRunning = intakeRunning;
    }

    public Distance getElevatorSetpoint() {
        return m_elevatorSetpoint;
    }

    public Rotation2d getArmSetpoint() {
        return m_armSetpoint;
    }

    public boolean getIntakeRunning() {
        return m_intakeRunning;
    }

    public CoralState running() {
        return new CoralState(m_elevatorSetpoint, m_armSetpoint, true);
    }
}