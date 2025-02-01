package frc.robot.controller;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;

/** implementation of GenericController for PID */
public class CustomPIDController extends GenericController<PIDController, Double> {
    private final Optional<Double> m_feedForward; // not sure about this...
    private final Optional<Double> m_desired;

    public CustomPIDController(PIDConstants pidConstants, Supplier<Double> getCurrentValue, double desired) {
        super(
            new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD),
            getCurrentValue
        );

        m_feedForward = pidConstants.kFF;
        m_desired = Optional.of(desired);
    }

    public CustomPIDController(PIDConstants pidConstants, Supplier<Double> getCurrentValue, double desired, Clamp<Double> clamp) {
        super(
            new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD),
            getCurrentValue,
            clamp
        );

        m_feedForward = pidConstants.kFF;
        m_desired = Optional.of(desired);
    }

    public CustomPIDController(PIDConstants pidConstants, Supplier<Double> getCurrentValue) {
        super(
            new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD),
            getCurrentValue
        );

        m_feedForward = pidConstants.kFF;
        m_desired = Optional.empty();
    }

    public CustomPIDController(PIDConstants pidConstants, Supplier<Double> getCurrentValue, Clamp<Double> clamp) {
        super(
            new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD),
            getCurrentValue,
            clamp
        );

        m_feedForward = pidConstants.kFF;
        m_desired = Optional.empty();
    }

    protected Double getOutput() {
        double calculatedVal;

        if (m_desired.isEmpty()) {
            calculatedVal = m_getCurrentValue.get(); // if no desired value then calculates to current value
        } else {
            calculatedVal = m_controller.calculate(m_getCurrentValue.get(), m_desired.get());
        }

        // ngl kinda confused here
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html
        if (m_feedForward.isEmpty()) {
            return calculatedVal;
        } else {
            return calculatedVal + m_feedForward.get();
        }
    }   
}
