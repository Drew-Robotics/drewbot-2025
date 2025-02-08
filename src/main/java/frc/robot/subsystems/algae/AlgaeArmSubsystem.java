package frc.robot.subsystems.algae;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.subsystems.Subsystem;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.AlgaeConstants.AlgaePivotConstants;
import frc.robot.constants.AlgaeConstants.AlgaePivotConstants.PivotPID;
import frc.robot.constants.DriveConstants;

public class AlgaeArmSubsystem extends Subsystem {
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
            AlgaeConstants.CANIDs.kPivot,
            MotorType.kBrushless
        );
        m_algaePivotEncoder = m_algaePivotMotorController.getAbsoluteEncoder();
        m_algaePivotClosedLoopController = m_algaePivotMotorController.getClosedLoopController();

        SparkFlexConfig algaePivotMotorConfig = new SparkFlexConfig();

        algaePivotMotorConfig
            .smartCurrentLimit(AlgaePivotConstants.kMaxAmps);
    
        algaePivotMotorConfig
            .closedLoop      
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(
                AlgaeConstants.PID.kP, 
                AlgaePivotConstants.PivotPID.kI, 
                AlgaePivotConstants.PivotPID.kD, 
                AlgaePivotConstants.PivotPID.kFF
            )
            .outputRange(-1,1);

        m_algaePivotMotorController.configure(algaePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setDesiredAngle(Rotation2d angle) {
        m_currentDesiredAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_algaePivotEncoder.getPosition());
    } 

    private void setMotor(double speed) {
        m_algaePivotMotorController.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_currentDesiredAngle != null) {
            double clampedRefrenceRot = MathUtil.clamp(
                m_currentDesiredAngle.getRotations(), 
                AlgaePivotConstants.kPivotMinPosition.getRotations(),
                AlgaePivotConstants.kPivotMaxPosition.getRotations()
            );

            m_algaePivotClosedLoopController.setReference(clampedRefrenceRot, SparkMax.ControlType.kPosition);
        }   
    }

    // Dashboard Fluff //
    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Current Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Desired Angle", m_currentDesiredAngle);
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
