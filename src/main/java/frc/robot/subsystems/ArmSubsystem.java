package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.ArmConstants;
import frc.robot.controller.Clamp;
import frc.robot.controller.CustomPIDController;
import frc.robot.controller.PIDConstants;

public class ArmSubsystem extends Subsystem {
  private final CustomPIDController m_armPID;
  private Rotation2d m_armMeasuredAngle;
  private Rotation2d m_armDesiredAngle;

  private SparkMax m_armMotorController;
  private SparkAbsoluteEncoder m_armEncoder;

  protected static ArmSubsystem m_instance;
  public static ArmSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new ArmSubsystem();
      return m_instance;
  }

  public ArmSubsystem() {
    super();

    m_armMotorController = new SparkMax(ArmConstants.ArmCANIDs.kArm, MotorType.kBrushless);

    m_armEncoder = m_armMotorController.getAbsoluteEncoder();
    m_armPID = new CustomPIDController(
      new PIDConstants(
        ArmConstants.ArmPID.kP,
        ArmConstants.ArmPID.kI,
        ArmConstants.ArmPID.kD
      ),
      () -> Rotation2d.fromRotations(m_armEncoder.getPosition()).getRadians(),
      new Clamp<Double>(
        ArmConstants.kArmMinPosition.getRadians(), 
        ArmConstants.kArmMaxPosition.getRadians()
      )
    );
  }

  @Override
  public void periodic() {
    setMotor(m_armPID.calculate());
  }

  private void setMotor(double speed) {
    m_armMotorController.set(speed);
  }

  public void setDesiredAngle(Rotation2d angle) {
    m_armPID.setDesiredValue(angle.getRadians());
  }

  // Dashboard Fluff //
  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Desired Arm Angle (degrees)", m_armDesiredAngle.getDegrees());
    SmartDashboard.putNumber("Measured Arm Angle (degrees)", m_armMeasuredAngle.getDegrees());
  }

  protected void dashboardInit() {}
  protected void publishInit() {}
  protected void publishPeriodic() {}
}
