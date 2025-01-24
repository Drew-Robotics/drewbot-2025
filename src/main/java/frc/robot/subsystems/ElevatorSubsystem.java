package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends Subsystem {

    private final CANSparkMax m_elevatorMotorController;

    public ElevatorSubsystem() {
        super();

        m_elevatorMotorController = new CANSparkMax(ElevatorConstants.ElevatorCANIDs.kElevator, MotorType.kBrushless);
        
    }

    public enum ElevatorLevels {
        L1,
        L2,
        L3,
        L4,
    }

    protected void dashboardInit() {}

    protected void dashboardPeriodic() {}

    protected void publishInit() {}

    protected void publishPeriodic() {}
}