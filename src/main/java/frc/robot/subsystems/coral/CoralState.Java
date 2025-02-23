package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class CoralState {

    private final Distance m_elevatorSetpoint;
    private final Rotation2d m_armSetpoint;

    public CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint) {
        m_elevatorSetpoint = elevatorSetpoint;
        m_armSetpoint = armSetpoint;
    }

    public Distance getElevatorSetpoint() {
        return m_elevatorSetpoint;
    }

    public Rotation2d getArmSetpoint() {
        return m_armSetpoint;
    }
}