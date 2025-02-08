package frc.robot.subsystems.coral;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.Map;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static java.util.Map.entry;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.Subsystem;

import frc.lib.LibFuncs;

public class ElevatorSubsystem extends Subsystem {

    private final SparkMax m_elevatorMotorControllerLeft;
    private final AbsoluteEncoder m_elevatorEncoderLeft;

    private final SparkMax m_elevatorMotorControllerRight;
    private final AbsoluteEncoder m_elevatorEncoderRight;

    private final SparkClosedLoopController m_elevatorClosedLoopControllerLeft;
    private final SparkClosedLoopController m_elevatorClosedLoopControllerRight;

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

        m_elevatorMotorControllerLeft = new SparkMax(CoralConstants.CANIDs.kElevatorLeft, MotorType.kBrushless);
        m_elevatorMotorControllerRight = new SparkMax(CoralConstants.CANIDs.kElevatorRight, MotorType.kBrushless);

        m_elevatorEncoderLeft = m_elevatorMotorControllerRight.getAbsoluteEncoder();
        m_elevatorEncoderRight = m_elevatorMotorControllerRight.getAbsoluteEncoder();

        m_elevatorClosedLoopControllerLeft = m_elevatorMotorControllerLeft.getClosedLoopController();
        m_elevatorClosedLoopControllerRight = m_elevatorMotorControllerRight.getClosedLoopController();
        
        SparkFlexConfig elevatorMotorControllerConfigLeft = new SparkFlexConfig();
        SparkFlexConfig elevatorMotorControllerConfigRight = new SparkFlexConfig();

        elevatorMotorControllerConfigLeft
            .smartCurrentLimit((int) CoralConstants.kElevatorCurrentLimit.in(Amps))
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .inverted(true);
        
        elevatorMotorControllerConfigLeft
            .limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);

        elevatorMotorControllerConfigLeft
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                CoralConstants.PID.Elevator.kP,
                CoralConstants.PID.Elevator.kI,
                CoralConstants.PID.Elevator.kD,
                CoralConstants.PID.Elevator.kFF
            )
            .outputRange(-1,1);

        elevatorMotorControllerConfigRight
            .smartCurrentLimit((int) CoralConstants.kElevatorCurrentLimit.in(Amps))
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12)
            .inverted(false);
        
        elevatorMotorControllerConfigRight
            .limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);

        elevatorMotorControllerConfigRight
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                CoralConstants.PID.Elevator.kP,
                CoralConstants.PID.Elevator.kI,
                CoralConstants.PID.Elevator.kD,
                CoralConstants.PID.Elevator.kFF
            )
            .outputRange(-1,1);
    }

    public void setLevel(ElevatorLevels level) {
        m_desiredHeight = levelHeights.get(level).in(Meters);

        m_elevatorLevelString = level.toString();
    }

    private void setMotorLeft(double speed) {
        m_elevatorMotorControllerLeft.set(speed);
    }

    private void setMotorRight(double speed) {
        m_elevatorMotorControllerLeft.set(speed);
    }

    private double getHeight() {
        Angle angularDistance = CoralConstants.ArmHeightConversion.kMaxRotations.plus(
            CoralConstants.ArmHeightConversion.kMinRotations.times(-1)
        );

        Distance heightDistance = CoralConstants.ArmHeightConversion.kElevatorMaxHeight.plus(
            CoralConstants.ArmHeightConversion.kElevatorMinHeight.times(-1)
        );

        double leftHeight = LibFuncs.sum( 
            LibFuncs.mult(
                LibFuncs.sum( 
                    m_elevatorEncoderLeft.getPosition(),
                    -CoralConstants.ArmHeightConversion.kMinRotations.in(Units.Radians)
                ) / angularDistance.in(Units.Radians),
                heightDistance.in(Meters)
            ),
            CoralConstants.ArmHeightConversion.kElevatorMinHeight.in(Meters)
        );

        double rightHeight = LibFuncs.sum( 
            LibFuncs.mult(
                LibFuncs.sum( 
                    m_elevatorEncoderRight.getPosition(),
                    -CoralConstants.ArmHeightConversion.kMinRotations.in(Units.Radians)
                ) / angularDistance.in(Units.Radians),
                heightDistance.in(Meters)
            ),
            CoralConstants.ArmHeightConversion.kElevatorMinHeight.in(Meters)
        );

        return (rightHeight + leftHeight) / 2; // getting average height
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_desiredHeight != null) { // wth
            double clampedRefrenceRot = MathUtil.clamp(
                m_desiredHeight, 
                CoralConstants.ArmHeightConversion.kElevatorMinHeight.in(Meters),
                CoralConstants.ArmHeightConversion.kElevatorMaxHeight.in(Meters)
            );

            Stream.of(
                m_elevatorClosedLoopControllerLeft,
                m_elevatorClosedLoopControllerRight
            ).forEach(controller -> 
                controller.setReference(clampedRefrenceRot, SparkMax.ControlType.kPosition));
        }   
    }

    public enum ElevatorLevels {
        L1,
        L2,
        L3,
        L4,
    }

    public static final Map<ElevatorLevels, Distance> levelHeights = Map.ofEntries(
        entry(ElevatorLevels.L1, CoralConstants.ReefHeights.L1),
        entry(ElevatorLevels.L2, CoralConstants.ReefHeights.L2),
        entry(ElevatorLevels.L3, CoralConstants.ReefHeights.L3),
        entry(ElevatorLevels.L4, CoralConstants.ReefHeights.L4)
    );

    // Dashboard Fluff //
    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        SmartDashboard.putString("Current Elevator Level", m_elevatorLevelString);
        SmartDashboard.putNumber("Current Height (Meters)", getHeight());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}