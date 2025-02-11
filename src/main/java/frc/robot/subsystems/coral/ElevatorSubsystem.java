package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class ElevatorSubsystem extends SubsystemAbstract implements CoralSubsystemI{

    private final SparkFlex m_elevatorLeadMotor;
    private final SparkFlex m_elevatorFollowerMotorRight;

    private final RelativeEncoder m_elevatorLeftEncoder;
    private final RelativeEncoder m_elevatorRightEncoder;

    private final SparkClosedLoopController m_elevatorClosedLoopController;

    private CoralState m_targetState;

    private boolean m_switchReset = false;

    protected static ElevatorSubsystem m_instance;
    public static ElevatorSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ElevatorSubsystem();
        return m_instance;
    }

    public ElevatorSubsystem() {
        super();
        m_elevatorLeadMotor = new SparkFlex(CoralConstants.CANIDs.kElevatorLeft, MotorType.kBrushless);
        m_elevatorFollowerMotorRight = new SparkFlex(CoralConstants.CANIDs.kElevatorRight, MotorType.kBrushless);

        m_elevatorLeftEncoder = m_elevatorLeadMotor.getEncoder();
        m_elevatorRightEncoder = m_elevatorFollowerMotorRight.getEncoder();

        m_elevatorClosedLoopController = m_elevatorLeadMotor.getClosedLoopController();
        
        SparkFlexConfig elevatorConfigLeader = new SparkFlexConfig();
        SparkFlexConfig elevatorConfigFollowerRight = new SparkFlexConfig();

        elevatorConfigLeader
            .smartCurrentLimit((int) CoralConstants.kElevatorCurrentLimit.in(Units.Amps))
            .idleMode(IdleMode.kCoast)
            .inverted(CoralConstants.kElevatorLeftMotorInverted);
        elevatorConfigLeader
            .encoder
            .positionConversionFactor(1) // we don't have a linear scaling system so we're doing something custom
            .velocityConversionFactor(1);
        elevatorConfigLeader
            .limitSwitch
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen);
        elevatorConfigLeader
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                CoralConstants.PID.Elevator.kP,
                CoralConstants.PID.Elevator.kI,
                CoralConstants.PID.Elevator.kD,
                CoralConstants.PID.Elevator.kFF
            )
            .outputRange(-1,1);
            

        elevatorConfigFollowerRight
            .smartCurrentLimit((int) CoralConstants.kElevatorCurrentLimit.in(Units.Amps))
            .idleMode(IdleMode.kCoast)
            .inverted(CoralConstants.kElevatorRightMotorInverted)
            .follow(m_elevatorLeadMotor);

        m_elevatorLeadMotor.configure(elevatorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorFollowerMotorRight.configure(elevatorConfigFollowerRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    /* ----- OVERIDES ----- */

    @Override
    public void periodic() {
        super.periodic();
        
        // reset from limit switch
        if (m_elevatorLeadMotor.getReverseLimitSwitch().isPressed() && !m_switchReset) {
            m_switchReset = true;

            m_elevatorLeftEncoder.setPosition(CoralConstants.ArmHeightConversion.kMaxRotations.in(Units.Rotations));
            m_elevatorRightEncoder.setPosition(CoralConstants.ArmHeightConversion.kMaxRotations.in(Units.Rotations));
        } 
        else if (!m_elevatorLeadMotor.getReverseLimitSwitch().isPressed()) {
            m_switchReset = false;
        }


        // set refrence and move the motor
        m_elevatorClosedLoopController.setReference(
            heightToRotations(m_targetState.getElevatorSetpoint()).getRotations(), // only time we're using rotations
            ControlType.kPosition
        );
    }

    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Elevator Encoder Reading", getEncoderReading().getRotations());
        SmartDashboard.putNumber("Elevator Height", getHeight().in(Units.Meters));
        SmartDashboard.putBoolean("Limit Switch Pressed", m_elevatorLeadMotor.getReverseLimitSwitch().isPressed());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}

    public void setState(CoralState state) {
        m_targetState = state;
    }

    public Rotation2d getEncoderReading() {
        Rotation2d leftHeight = Rotation2d.fromRotations(m_elevatorLeftEncoder.getPosition());

        Rotation2d rightHeight = Rotation2d.fromRotations(m_elevatorRightEncoder.getPosition());

        return leftHeight.plus(rightHeight).times(0.5);
    }

    public Distance getHeight() {
        return rotationsToHeight(getEncoderReading());
    }

    /* ----- CONVERSIONS ----- */

    private Distance rotationsToHeight(Rotation2d rot) {
        Angle angularDistance = CoralConstants.ArmHeightConversion.kMaxRotations.plus(
            CoralConstants.ArmHeightConversion.kMinRotations.times(-1)
        );

        Distance heightDistance = CoralConstants.ArmHeightConversion.kElevatorMaxHeight.plus(
            CoralConstants.ArmHeightConversion.kElevatorMinHeight.times(-1)
        );

        double progression = 
            (rot.getRadians() - CoralConstants.ArmHeightConversion.kMinRotations.in(Units.Radians))
                / angularDistance.in(Units.Radians);
        

        double heightMeters = progression * heightDistance.in(Units.Meters) 
            + CoralConstants.ArmHeightConversion.kElevatorMinHeight.in(Units.Meters);

        return Units.Meters.of(heightMeters);
    }

    private Rotation2d heightToRotations(Distance height) {
        Angle angularDistance = CoralConstants.ArmHeightConversion.kMaxRotations.plus(
            CoralConstants.ArmHeightConversion.kMinRotations.times(-1)
        );

        Distance heightDistance = CoralConstants.ArmHeightConversion.kElevatorMaxHeight.plus(
            CoralConstants.ArmHeightConversion.kElevatorMinHeight.times(-1)
        );

        double progression = 
            (height.in(Units.Meters) - CoralConstants.ArmHeightConversion.kElevatorMinHeight.in(Units.Meters))
                / heightDistance.in(Units.Meters);
        

        double angleRadians = progression * angularDistance.in(Units.Radians) 
            + CoralConstants.ArmHeightConversion.kMinRotations.in(Units.Radians);

        return Rotation2d.fromRadians(angleRadians);
    }

}