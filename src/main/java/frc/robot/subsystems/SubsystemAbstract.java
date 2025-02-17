package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemAbstract extends SubsystemBase {
    
    protected final String m_name;
    protected final NetworkTable m_table;

    // private List<TopicSupAbstract> m_topicSups = List.of();

    protected abstract void dashboardInit();
    protected abstract void dashboardPeriodic();

    protected abstract void publishInit();
    protected abstract void publishPeriodic();

    protected SubsystemAbstract() {
        m_name = this.getClass().getSimpleName();
        m_table = NetworkTableInstance.getDefault().getTable(m_name);

        publishInit();
        dashboardInit();
    }

    // public TopicSupAbstract addTopicSup(TopicSupAbstract topicSup) {
    //     m_topicSups.add(topicSup);
    //     return topicSup;
    // }

    @Override
    public void periodic() {
        super.periodic();

        // m_topicSups.forEach(topicSup -> topicSup.update());

        publishPeriodic(); 
        dashboardPeriodic();
    }
}