package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.Map;
import static java.util.Map.entry;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Measurements;
import frc.robot.controller.CustomPIDController;

public class ElevatorSubsystem extends Subsystem {

    private final SparkMax m_elevatorMotorController;
    private 

    private final CustomPIDController pidController;

    public ElevatorSubsystem() {
        super();

        m_elevatorMotorController = new SparkMax(ElevatorConstants.ElevatorCANIDs.kElevator, MotorType.kBrushless);

        
    }

    public enum ElevatorLevels {
        L1,
        L2,
        L3,
        L4,
    }

    public static final Map<ElevatorLevels, Double> levelHeights = Map.ofEntries(
        entry(ElevatorLevels.L1, Measurements.L1),
        entry(ElevatorLevels.L2, Measurements.L2),
        entry(ElevatorLevels.L3, Measurements.L3),
        entry(ElevatorLevels.L4, Measurements.L4)
    );

    protected void dashboardInit() {}

    protected void dashboardPeriodic() {}

    protected void publishInit() {}

    protected void publishPeriodic() {}
}