package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Measurements;
import frc.robot.controller.Clamp;
import frc.robot.controller.CustomPIDController;

public class ElevatorSubsystem extends Subsystem {

    private final SparkMax m_elevatorMotorController;
    private final AbsoluteEncoder m_elevatorEncoder;

    private final CustomPIDController m_elevatorPID;

    private String m_elevatorLevelString = "RESTING";

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
            this::getHeight,
            new Clamp<Double>(
                ElevatorConstants.kElevatorMinHeight.in(Meters), 
                ElevatorConstants.kElevatorMaxHeight.in(Meters)
            )
        );
    }

    public void setLevel(ElevatorLevels level) {
        m_elevatorPID.setDesiredValue(levelHeights.get(level));

        m_elevatorLevelString = level.toString();
    }

    private void setMotor(double speed) {
        m_elevatorMotorController.set(speed);
    }

    private double getHeight() {
        return m_elevatorEncoder.getPosition() * 2 * Math.PI; // idk what to do here ngl (hopefully this is somewhat close?)
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

    protected void dashboardPeriodic() {
        SmartDashboard.putString("Current Elevator Level", m_elevatorLevelString);
        SmartDashboard.putNumber("Current Height", getHeight());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}