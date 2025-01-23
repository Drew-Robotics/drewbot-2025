package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    
    protected final String m_name;
    protected final NetworkTable m_table;

    protected static Subsystem m_instance;

    protected Subsystem() {
        m_name = this.getSimpleName();
        m_table = NetworkTableInstance.getDefault().getTable(m_name);

        publishInit();
        dashboardInit();
    }

    public static Subsystem getInstance() {
        if (m_instance == null)
            m_instance = new this();
        return m_instance;
    }

    @Override
    public void periodic() {
        super.periodic();
        publishPeriodic();
        dashboardPeriodic();
    }
}
