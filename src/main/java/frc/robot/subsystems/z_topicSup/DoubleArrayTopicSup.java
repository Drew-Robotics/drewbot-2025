package frc.robot.subsystems.z_topicSup;

import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleArrayPublisher;

public class DoubleArrayTopicSup extends TopicSupAbstract<Double[]> {
    private DoubleArrayPublisher m_publisher;
    private Supplier<double[]> m_supplier;

    public DoubleArrayTopicSup(DoubleArrayPublisher publisher, double[] initialValue) {
        m_publisher = publisher;
        m_publisher.accept(initialValue);
    }

    public DoubleArrayTopicSup(DoubleArrayPublisher publisher, Supplier<double[]> supplier) {
        m_publisher = publisher;
        m_supplier = supplier;
    }

    public void update() {
        if (m_supplier == null)
            return;
        
        m_publisher.accept(m_supplier.get());
    }

    public DoubleArrayPublisher getPublisher() {
        return m_publisher;
    }
}
