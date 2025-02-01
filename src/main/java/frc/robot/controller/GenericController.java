package frc.robot.controller;

import java.util.Optional;
import java.util.function.Supplier;

public abstract class GenericController<T, U extends Comparable<U>> {
    protected final T m_controller;
    protected final Optional<Clamp<U>> m_clamp;

    protected final Supplier<U> m_getCurrentValue;
    protected Optional<U> m_desired;

    /**
     * Sets method of output
     */
    protected abstract U getOutput();

    public GenericController(T controller, Supplier<U> getCurrentValue) {
        m_controller = controller;

        m_clamp = Optional.empty();
        
        m_getCurrentValue = getCurrentValue;
    }

    public GenericController(T controller, Supplier<U> getCurrentValue, Clamp<U> clamp) {
        m_controller = controller;

        m_clamp = Optional.of(clamp);
        
        m_getCurrentValue = getCurrentValue;
    }

    public void setDesiredValue(U desired) {
        m_desired = Optional.of(desired);
    }

    public U calculate() {
        if (m_clamp.isEmpty()) {
            return getOutput();
        } else {
            var clamp = m_clamp.get();
            var output = getOutput();

            if (output.compareTo(clamp.m_min) < 0) {  // output < m_min
                return clamp.m_min;
            } else if (output.compareTo(clamp.m_max) > 0) {  // output > m_max
                return clamp.m_max;
            } else {
                return output;
            }
        }
    }
}
