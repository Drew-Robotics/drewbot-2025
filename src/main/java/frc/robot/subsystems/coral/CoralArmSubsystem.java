package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.CoralArmConstants;
import frc.robot.subsystems.Subsystem;

public class CoralArmSubsystem extends Subsystem {
  private final SparkMax m_coralPivotMotorController;
  private final SparkAbsoluteEncoder m_coralPivotEncoder;
  private final SparkClosedLoopController m_coralPivotClosedLoopController;

  private Rotation2d m_armDesiredAngle;

  protected static CoralArmSubsystem m_instance;
  public static CoralArmSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new CoralArmSubsystem();
      return m_instance;
  }

  public CoralArmSubsystem() {
    super();

    m_coralPivotMotorController = new SparkMax(CoralArmConstants.ArmCANIDs.kArm, MotorType.kBrushless);
    m_coralPivotEncoder = m_coralPivotMotorController.getAbsoluteEncoder();
    m_coralPivotClosedLoopController = m_coralPivotMotorController.getClosedLoopController();
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_coralPivotEncoder.getPosition());
  }

  private void setMotor(double speed) {
    m_coralPivotMotorController.set(speed);
  }

  public void setDesiredAngle(Rotation2d angle) {
    m_coralPivotClosedLoopController.setDesiredValue(angle.getRadians());
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
