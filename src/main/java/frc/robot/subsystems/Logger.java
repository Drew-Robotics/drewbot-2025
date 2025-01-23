package frc.robot.subsystems;

public interface Logger {
    default public void dashboardInit() {}
    default public void dashboardPeriodic() {}

    default public void publishInit() {}
    default public void publishPeriodic() {}
}