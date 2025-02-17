package frc.robot.subsystems;

public interface LoggerI {
    default void publishPeriodic() {};
    default void publishInit() {};
}
