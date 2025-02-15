package frc.robot.subsystems.algae;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import com.playingwithfusion.TimeOfFlight;

import frc.robot.constants.AlgaeConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class AlgaeSensorSubsystem extends SubsystemAbstract {
    private final TimeOfFlight m_algaeSensor;

    protected static AlgaeSensorSubsystem m_instance;
    public static AlgaeSensorSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new AlgaeSensorSubsystem();
        return m_instance;
    }

    public AlgaeSensorSubsystem() {
        m_algaeSensor = new TimeOfFlight(0);
    }

    private Distance getSenorRange() {
        return Units.Millimeters.of(m_algaeSensor.getRange());
    }

    public boolean detecting() {
        return 
            getSenorRange().in(Units.Meters) <= 
                AlgaeConstants.kSensorTripDistance.in(Units.Meters);
    }

    /* Overrides */
    protected void dashboardInit() {}
    
    protected void dashboardPeriodic() {}

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
