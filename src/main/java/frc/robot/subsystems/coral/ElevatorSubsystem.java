package frc.robot.subsystems.coral;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ElevatorConstants.Measurements;
import frc.robot.subsystems.Subsystem;

import lib.LibFuncs;

public class ElevatorSubsystem extends Subsystem {

    private final SparkMax m_elevatorMotorController;
    private final AbsoluteEncoder m_elevatorEncoder;
    private final SparkClosedLoopController m_elevatorClosedLoopController;

    private String m_elevatorLevelString = "RESTING";

    private double m_desiredHeight;

    protected static ElevatorSubsystem m_instance;
    public static ElevatorSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ElevatorSubsystem();
        return m_instance;
    }

    public ElevatorSubsystem() {
        super();

        m_elevatorMotorController = new SparkMax(ElevatorConstants.ElevatorCANIDs.kElevator, MotorType.kBrushless);
        m_elevatorEncoder = m_elevatorMotorController.getAbsoluteEncoder();
        m_elevatorClosedLoopController = m_elevatorMotorController.getClosedLoopController();
        
        SparkFlexConfig elevatorMotorControllerConfig = new SparkFlexConfig();

        elevatorMotorControllerConfig
            .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12);
        
        elevatorMotorControllerConfig
            .limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);

        elevatorMotorControllerConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                ElevatorConstants.ElevatorPID.kP,
                ElevatorConstants.ElevatorPID.kI,
                ElevatorConstants.ElevatorPID.kD,
                ElevatorConstants.ElevatorPID.kFF
            )
            .outputRange(-1,1);
    }

    public void setLevel(ElevatorLevels level) {
        m_desiredHeight = levelHeights.get(level);

        m_elevatorLevelString = level.toString();
    }

    private void setMotor(double speed) {
        m_elevatorMotorController.set(speed);
    }

    private double getHeight() {
        Angle angularDistance = ElevatorConstants.kMaxRotations.plus(
            ElevatorConstants.kMinRotations.times(-1)
        );

        Distance heightDistance = ElevatorConstants.kElevatorMaxHeight.plus(
            ElevatorConstants.kElevatorMinHeight.times(-1)
        );

        return LibFuncs.sum( 
            LibFuncs.mult(
                LibFuncs.sum( 
                    m_elevatorEncoder.getPosition(),
                    -ElevatorConstants.kMinRotations.in(Units.Radians)
                ) / angularDistance.in(Units.Radians),
                heightDistance.in(Meters)
            ),
            ElevatorConstants.kElevatorMinHeight.in(Meters)
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_desiredHeight != null) {
            double clampedRefrenceRot = MathUtil.clamp(
                m_desiredHeight, 
                ElevatorConstants.kElevatorMinHeight,
                ElevatorConstants.kElevatorMaxHeight
            );

            m_algaePivotClosedLoopController.setReference(clampedRefrenceRot, SparkMax.ControlType.kPosition);
        }   
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