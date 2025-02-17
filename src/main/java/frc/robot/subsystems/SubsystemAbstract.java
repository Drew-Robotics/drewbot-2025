package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemAbstract extends SubsystemBase {
    
    protected final String m_name;
    protected final NetworkTable m_table;
    protected final LoggerI m_logger;

    protected SubsystemAbstract(LoggerI logger) {
        m_name = this.getClass().getSimpleName();
        m_table = NetworkTableInstance.getDefault().getTable(m_name);

        m_logger = logger;

        m_logger.publishInit();
    }

    @Override
    public void periodic() {
        super.periodic();

        m_logger.publishPeriodic(); 
    }
}