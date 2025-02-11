package frc.robot.subsystems.topicSup;

import edu.wpi.first.networktables.Publisher;

public abstract class TopicSupAbstract<T> {
    public abstract void update();
    public abstract Publisher getPublisher();
}
