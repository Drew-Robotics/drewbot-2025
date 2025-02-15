package frc.robot.subsystems.coral;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeStates;

public class CoralState {

    private final Distance m_elevatorSetpoint;
    private final Rotation2d m_armSetpoint;
    private CoralIntakeStates m_intakeState;

    public CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint) {
        this(elevatorSetpoint, armSetpoint, CoralIntakeStates.Rest);
    }

    public CoralState(Distance elevatorSetpoint, Rotation2d armSetpoint, CoralIntakeStates intakeState) {
        m_elevatorSetpoint = elevatorSetpoint;
        m_armSetpoint = armSetpoint;
        m_intakeState = intakeState;
    }

    public Distance getElevatorSetpoint() {
        return m_elevatorSetpoint;
    }

    public Rotation2d getArmSetpoint() {
        return m_armSetpoint;
    }

    public CoralIntakeStates getIntakeState() {
        return m_intakeState;
    }

    public CoralState setIntakeState(CoralIntakeStates intakeState) {
        return new CoralState(m_elevatorSetpoint, m_armSetpoint, intakeState);
    }

    public CoralState intake() {
        return new CoralState(m_elevatorSetpoint, m_armSetpoint, CoralIntakeStates.Intake);
    }

    public CoralState outtake() {
        return new CoralState(m_elevatorSetpoint, m_armSetpoint, CoralIntakeStates.Outtake);
    }
}