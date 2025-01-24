package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends Subsystem {
  private final SparkPIDController m_armPID;
  private Rotation2d m_armMeasuredAngle;
  private Rotation2d m_armDesiredAngle;

  private CANSparkMax m_armMotorController;
  private SparkAbsoluteEncoder m_armEncoder;

  protected static ArmSubsystem m_instance;
  public static ArmSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new ArmSubsystem();
      return m_instance;
  }

  public ArmSubsystem() {
    super();

    m_armMotorController = new CANSparkMax(ArmConstants.ArmCANIDs.kArm, MotorType.kBrushless);

    m_armEncoder = m_armMotorController.getAbsoluteEncoder();
    m_armPID = m_armMotorController.getPIDController();

    m_armPID.setP(ArmConstants.ArmPID.kP);
    m_armPID.setI(ArmConstants.ArmPID.kI);
    m_armPID.setD(ArmConstants.ArmPID.kD);
    m_armPID.setFF(ArmConstants.ArmPID.kFF);

    m_armPID.setOutputRange(
      ArmConstants.kArmMinPosition.getRadians(),
      ArmConstants.kArmMaxPosition.getRadians()
    );
    
    m_armPID.setFeedbackDevice(m_armEncoder);
  }

  @Override
  public void periodic() {
    m_armMeasuredAngle = Rotation2d.fromRotations(m_armEncoder.getPosition());
    
    updatePIDController();
  }

  private void updatePIDController() {
    m_armPID.setReference(m_armDesiredAngle.getRadians(), ControlType.kPosition);
  }

  public void setDesiredAngle(Rotation2d angle) {
    m_armDesiredAngle = angle;
  }

  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Desired Arm Angle (degrees)", m_armDesiredAngle.getDegrees());
    SmartDashboard.putNumber("Measured Arm Angle (degrees)", m_armMeasuredAngle.getDegrees());
  }

  protected void dashboardInit() {}

  protected void publishInit() {}
  
  protected void publishPeriodic() {}
}
