package frc.robot.subsystems.coral;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.constants.CoralConstants;

public class CoralArmSubsystem extends SubsystemAbstract implements CoralSubsystemI{
  private final SparkFlex m_coralPivotMotorController;
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

    m_coralPivotMotorController = new SparkFlex(CoralConstants.CANIDs.kCoralArm, MotorType.kBrushless);
    m_coralPivotEncoder = m_coralPivotMotorController.getAbsoluteEncoder();
    m_coralPivotClosedLoopController = m_coralPivotMotorController.getClosedLoopController();

    SparkFlexConfig coralPivotConfig = new SparkFlexConfig();

    coralPivotConfig
      .smartCurrentLimit((int) CoralConstants.kArmCurrentLimit.in(Units.Amps));

    coralPivotConfig
      .absoluteEncoder
      .positionConversionFactor(CoralConstants.ConversionFactors.Arm.kPositionConversionFactor.in(Units.Radians))
      .velocityConversionFactor(CoralConstants.ConversionFactors.Arm.kVelocityConversionFactor.in(Units.RadiansPerSecond));

    coralPivotConfig
      .closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pidf( 
        CoralConstants.PID.CoralArm.kP,
        CoralConstants.PID.CoralArm.kI,
        CoralConstants.PID.CoralArm.kD,
        CoralConstants.PID.CoralArm.kFF
      )
      .outputRange(-1, 1);

      m_coralPivotMotorController.configure(coralPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setDesiredAngle(Rotation2d angle) {
    m_armDesiredAngle = angle;

    m_coralPivotClosedLoopController.setReference(clampAngle(angle).getRotations(), ControlType.kPosition);
  }

  private Rotation2d clampAngle(Rotation2d angle) {
    return Rotation2d.fromRadians(
      MathUtil.clamp(
        angle.getRadians(),
        CoralConstants.kMinArmAngle.getRadians(),
        CoralConstants.kMaxArmAngle.getRadians()
      )
    );
  }

  /* Getters and Setters */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(m_coralPivotEncoder.getPosition());
  }

  public void setState(CoralState state) {
    setDesiredAngle(state.getArmSetpoint());
  }

  /* Overrides */
  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Desired Arm Angle (degrees)", m_armDesiredAngle.getDegrees());
    SmartDashboard.putNumber("Measured Arm Angle (degrees)", getAngle().getDegrees());
  }

  protected void dashboardInit() {}
  protected void publishInit() {}
  protected void publishPeriodic() {}
}