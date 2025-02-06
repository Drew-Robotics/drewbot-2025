package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends Subsystem {
  private final CustomPIDController m_armPID;
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
      () -> getAngle().getRadians(),
      new Clamp<Double>(
        ArmConstants.kArmMinPosition.getRadians(), 
        ArmConstants.kArmMaxPosition.getRadians()
      )
    );
  }

  @Override
  public void periodic() {
    super.periodic();

    setMotor(m_armPID.calculate());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_armEncoder.getPosition());
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
    SmartDashboard.putNumber("Measured Arm Angle (degrees)", getAngle().getDegrees());
  }

  protected void dashboardInit() {}
  protected void publishInit() {}
  protected void publishPeriodic() {}
}
