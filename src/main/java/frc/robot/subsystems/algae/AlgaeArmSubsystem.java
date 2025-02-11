package frc.robot.subsystems.algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.constants.AlgaeConstants;

public class AlgaeArmSubsystem extends SubsystemAbstract {
    private final SparkFlex m_algaePivotMotorController;
    private final AbsoluteEncoder m_algaePivotEncoder;

    private final SparkClosedLoopController m_algaePivotClosedLoopController;

    private Rotation2d m_currentDesiredAngle;

    protected static AlgaeArmSubsystem m_instance;
    public static AlgaeArmSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new AlgaeArmSubsystem();
        return m_instance;
    }

    public AlgaeArmSubsystem() {
        super();
        
        m_algaePivotMotorController = new SparkFlex(
            AlgaeConstants.CANIDs.kArm,
            MotorType.kBrushless
        );
        m_algaePivotEncoder = m_algaePivotMotorController.getAbsoluteEncoder();
        m_algaePivotClosedLoopController = m_algaePivotMotorController.getClosedLoopController();

        SparkFlexConfig algaePivotMotorConfig = new SparkFlexConfig();

        algaePivotMotorConfig
            .smartCurrentLimit((int) AlgaeConstants.kArmCurrentLimit.in(Units.Amps));

        algaePivotMotorConfig
            .absoluteEncoder
            .positionConversionFactor(AlgaeConstants.ConversionFactors.Arm.kPositionConversionFactor.in(Units.Radians))
            .velocityConversionFactor(AlgaeConstants.ConversionFactors.Arm.kVelocityConversionFactor.in(Units.RadiansPerSecond));
    
        algaePivotMotorConfig
            .closedLoop      
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                AlgaeConstants.PID.Arm.kP, 
                AlgaeConstants.PID.Arm.kI, 
                AlgaeConstants.PID.Arm.kD, 
                AlgaeConstants.PID.Arm.kFF
            )
            .outputRange(-1,1);

        m_algaePivotMotorController.configure(algaePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* Getters and Setters */
    public void setDesiredAngle(Rotation2d angle) {
        m_currentDesiredAngle = angle;

        m_algaePivotClosedLoopController.setReference(clampAngle(angle).getRadians(), SparkMax.ControlType.kPosition);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_algaePivotEncoder.getPosition());
    }

    public Rotation2d clampAngle(Rotation2d angle) {
        return Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(), 
                AlgaeConstants.kPivotMinPosition.getRadians(),
                AlgaeConstants.kPivotMaxPosition.getRadians()
            )
        );
    }

    /* Overrides */
    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Current Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Desired Angle", m_currentDesiredAngle.getDegrees());
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
