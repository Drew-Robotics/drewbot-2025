package frc.robot.subsystems.topicSup;

import java.util.function.Supplier;
import edu.wpi.first.networktables.DoublePublisher;

public class DoubleTopicSup extends TopicSupAbstract<Double> {
    private DoublePublisher m_publisher;
    private Supplier<Double> m_supplier;

    public DoubleTopicSup(DoublePublisher publisher, Double initialValue) {
        m_publisher = publisher;
        m_publisher.accept(initialValue);
    }

    public DoubleTopicSup(DoublePublisher publisher, Supplier<Double> supplier) {
        m_publisher = publisher;
        m_supplier = supplier;
    }

    public void update() {
        if (m_supplier == null)
            return;
        
        m_publisher.accept(m_supplier.get());
    }
    
    public DoublePublisher getPublisher() {
        return m_publisher;
    }
}
