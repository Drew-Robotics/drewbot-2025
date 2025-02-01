package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import java.util.Map;
import static java.util.Map.entry;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Measurements;
import frc.robot.controller.CustomPIDController;

public class ElevatorSubsystem extends Subsystem {

    private final SparkMax m_elevatorMotorController;
    private final AbsoluteEncoder m_elevatorEncoder;

    private final CustomPIDController m_elevatorPID;

    protected static ArmSubsystem m_instance;
    public static ArmSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ArmSubsystem();
        return m_instance;
    }

    public ElevatorSubsystem() {
        super();

        m_elevatorMotorController = new SparkMax(ElevatorConstants.ElevatorCANIDs.kElevator, MotorType.kBrushless);
        m_elevatorEncoder = m_elevatorMotorController.getAbsoluteEncoder();
        
        m_elevatorPID = new CustomPIDController(
            ElevatorConstants.ElevatorPID.pidConstants, 
            () -> m_elevatorEncoder.getPosition()
        );
    }

    public void setLevel(ElevatorLevels level) {
        m_elevatorPID.setDesiredValue(levelHeights.get(level));
    }

    private void setMotor(double speed) {
        m_elevatorMotorController.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();

        setMotor(m_elevatorPID.calculate());
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

    // Dashboard Fluff //
    protected void dashboardInit() {}
    protected void dashboardPeriodic() {}
    protected void publishInit() {}
    protected void publishPeriodic() {}
}