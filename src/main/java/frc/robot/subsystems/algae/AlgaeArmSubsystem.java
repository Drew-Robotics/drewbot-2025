package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CoralConstants;

public class AlgaeArmSubsystem extends SubsystemAbstract {
    private final SparkFlex m_algaeArmMotorController;
    private final RelativeEncoder m_algaeArmEncoder;

    private double m_restTimer = Timer.getFPGATimestamp();
    private boolean m_restState = true;

    private final SparkClosedLoopController m_algaeArmClosedLoopController;

    private Rotation2d m_currentDesiredAngle = Rotation2d.kZero; // TODO: fix this

    protected static AlgaeArmSubsystem m_instance;
    public static AlgaeArmSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new AlgaeArmSubsystem();
        return m_instance;
    }

    public AlgaeArmSubsystem() {
        super();
        
        m_algaeArmMotorController = new SparkFlex(
            AlgaeConstants.CANIDs.kArm,
            MotorType.kBrushless
        );

        m_algaeArmEncoder = m_algaeArmMotorController.getEncoder();
        m_algaeArmClosedLoopController = m_algaeArmMotorController.getClosedLoopController();

        SparkFlexConfig algaeArmMotorConfig = new SparkFlexConfig();

        algaeArmMotorConfig
            .smartCurrentLimit((int) AlgaeConstants.kArmCurrentLimit.in(Units.Amps))
            .inverted(AlgaeConstants.kAlgaeArmMotorInverted)
            .idleMode(AlgaeConstants.IdleModes.kArm);

        algaeArmMotorConfig
            .encoder
            .positionConversionFactor(AlgaeConstants.ConversionFactors.Arm.kPositionConversionFactor.in(Units.Radians))
            .velocityConversionFactor(AlgaeConstants.ConversionFactors.Arm.kVelocityConversionFactor.in(Units.RadiansPerSecond));
    
        algaeArmMotorConfig
            .closedLoop      
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                AlgaeConstants.PID.Arm.kP, 
                AlgaeConstants.PID.Arm.kI, 
                AlgaeConstants.PID.Arm.kD
            )
            .outputRange(-AlgaeConstants.PID.Arm.kOutput, AlgaeConstants.PID.Arm.kOutput);

        m_algaeArmMotorController.configure(algaeArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setEncoderZero() {
        m_algaeArmEncoder.setPosition(0);
    }

    /* Getters and Setters */
    public void setDesiredAngle(Rotation2d angle) {
        // System.out.println("not rest");
        m_restState = false;
        m_currentDesiredAngle = angle;

        m_algaeArmClosedLoopController.setReference(m_currentDesiredAngle.getRadians(), SparkMax.ControlType.kPosition);
    }

    public void toRestState() {
        // System.out.println("rest");
        m_restTimer = Timer.getFPGATimestamp();
        m_restState = true;
        m_algaeArmClosedLoopController.setReference(AlgaeConstants.kArmRestAngle.getRadians(), SparkMax.ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_algaeArmEncoder.getPosition());
    }

    /* Overrides */
    @Override
    public void periodic() {
        super.periodic();

        boolean restTimerActive = (Timer.getFPGATimestamp() - m_restTimer > AlgaeConstants.kRestTimerSeconds);

        // SmartDashboard.putBoolean("Algae Arm Timer", restTimerActive);
        // SmartDashboard.putBoolean("Algae Arm State", m_restState);
        if (m_restState && restTimerActive) {
            m_algaeArmClosedLoopController.setReference(AlgaeConstants.kArmRestVoltage.in(Units.Volts), ControlType.kVoltage);
            setEncoderZero();
        }
    }

    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        // SmartDashboard.putNumber("Algae Arm Desired Angle", m_currentDesiredAngle.getDegrees());
        // SmartDashboard.putNumber("Algae Arm Measured Angle", getAngle().getDegrees());
        // SmartDashboard.putNumber("Algae Arm Current", m_algaeArmMotorController.getOutputCurrent());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
