package frc.robot.subsystems.z_topicSup;

import java.util.function.Supplier;
import edu.wpi.first.networktables.StructArrayPublisher;

public class StructArrayTopicSup<T> extends TopicSupAbstract<T[]> {
    private StructArrayPublisher<T> m_publisher;
    private Supplier<T[]> m_supplier;

    public StructArrayTopicSup(StructArrayPublisher<T> publisher, T[] initialValue) {
        m_publisher = publisher;
        m_publisher.accept(initialValue);
    }

    public StructArrayTopicSup(StructArrayPublisher<T> publisher, Supplier<T[]> supplier) {
        m_publisher = publisher;
        m_supplier = supplier;
    }

    public void update() {
        if (m_supplier == null)
            return;
        
        m_publisher.accept(m_supplier.get());
    }

    public StructArrayPublisher<T> getPublisher() {
        return m_publisher;
    }
}
