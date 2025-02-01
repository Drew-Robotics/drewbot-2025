package frc.robot.controller;

public class Clamp<T> {
    public final T m_max;
    public final T m_min;

    public Clamp(T max, T min) {
        m_max = max;
        m_min = min;
    }
}
