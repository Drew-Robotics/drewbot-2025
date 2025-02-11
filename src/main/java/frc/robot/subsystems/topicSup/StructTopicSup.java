package frc.robot.subsystems.topicSup;

import java.util.function.Supplier;
import edu.wpi.first.networktables.StructPublisher;

public class StructTopicSup<T> extends TopicSupAbstract<T> {
    private StructPublisher<T> m_publisher;
    private Supplier<T> m_supplier;

    public StructTopicSup(StructPublisher<T> publisher, T initialValue) {
        m_publisher = publisher;
        m_publisher.accept(initialValue);
    }

    public StructTopicSup(StructPublisher<T> publisher, Supplier<T> supplier) {
        m_publisher = publisher;
        m_supplier = supplier;
    }

    public void update() {
        if (m_supplier == null)
            return;
        
        m_publisher.accept(m_supplier.get());
    }
    
    public StructPublisher<T> getPublisher() {
        return m_publisher;
    }
}
