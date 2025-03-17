package frc.robot.subsystems.coral;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.CoralStates;

public class CoralArmSubsystem extends SubsystemAbstract{
  private final SparkFlex m_coralArmMotorController;
  private final SparkMax m_coralArmEncoderController;
  private final SparkAbsoluteEncoder m_coralArmEncoder;

  private final ProfiledPIDController m_coralArmPIDController;
  private final ArmFeedforward m_coralArmFeedforwardController;
  // private final SparkClosedLoopController m_coralArmClosedLoopController;

  private CoralState m_targetState = CoralStates.kRest;
  private Rotation2d m_armDesiredAngle = CoralStates.kRest.getArmSetpoint();

  protected static CoralArmSubsystem m_instance;
  public static CoralArmSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new CoralArmSubsystem();
      return m_instance;
  }

  public CoralArmSubsystem() {
    super();

    m_coralArmMotorController = new SparkFlex(CoralConstants.CANIDs.kCoralArm, MotorType.kBrushless);
    m_coralArmEncoderController = new SparkMax(CoralConstants.CANIDs.kCoralArmEncoder, MotorType.kBrushless);

    m_coralArmEncoder = m_coralArmEncoderController.getAbsoluteEncoder();

    m_coralArmPIDController = new ProfiledPIDController(
      Math.abs(CoralConstants.PID.CoralArm.kP),
      Math.abs(CoralConstants.PID.CoralArm.kI),
      Math.abs(CoralConstants.PID.CoralArm.kD),
      new Constraints(
        CoralConstants.PID.CoralArm.kMaxVel.in(Units.RadiansPerSecond), 
        CoralConstants.PID.CoralArm.kMaxAccel.in(Units.RadiansPerSecondPerSecond)
      )
    );
    // m_coralArmPIDController.enableContinuousInput(
    //   CoralConstants.kArmMinAngle.in(Units.Radians),
    //   CoralConstants.kArmMaxAngle.in(Units.Radians)
    // );
    // m_coralArmClosedLoopController = m_coralArmEncoderController.getClosedLoopController();

    m_coralArmFeedforwardController = new ArmFeedforward(
      0,
      Math.abs(CoralConstants.PID.CoralArm.kG),
      0
    );

    SparkFlexConfig coralArmConfig = new SparkFlexConfig();
    SparkMaxConfig coralEncoderConfig = new SparkMaxConfig();

    coralArmConfig
      .smartCurrentLimit((int) CoralConstants.kArmCurrentLimit.in(Units.Amps))
      .idleMode(CoralConstants.IdleModes.kArm)
      .inverted(CoralConstants.kCoralArmMotorInverted);

    coralEncoderConfig
      .absoluteEncoder
      .positionConversionFactor(CoralConstants.ConversionFactors.Arm.kPositionConversionFactor.in(Units.Radians))
      .velocityConversionFactor(CoralConstants.ConversionFactors.Arm.kVelocityConversionFactor.in(Units.RadiansPerSecond));

    // coralEncoderConfig
    //   .closedLoop
    //   .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    //   .pidf( 
    //     CoralConstants.PID.CoralArm.kP,
    //     CoralConstants.PID.CoralArm.kI,
    //     CoralConstants.PID.CoralArm.kD,
    //     CoralConstants.PID.CoralArm.kFF
    //   )
    //   .positionWrappingEnabled(true)
    //   .positionWrappingInputRange(0, CoralConstants.ConversionFactors.Arm.kPositionConversionFactor.in(Units.Radians))
    //   .outputRange(-1, 1);


    m_coralArmMotorController.configure(coralArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralArmEncoderController.configure(coralEncoderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setDesiredAngle(Rotation2d angle) {
    m_armDesiredAngle = clampAngle(angle);
    // m_coralArmClosedLoopController.setReference(m_armDesiredAngle.getRadians(), ControlType.kPosition);
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
    return Rotation2d.fromRadians(m_coralArmEncoder.getPosition())
      .minus(Rotation2d.fromDegrees(63.42711));
  }

  public void setState(CoralState state) {
    m_targetState = state;
    setDesiredAngle(m_targetState.getArmSetpoint());
  }

  public CoralState getState() {
    return m_targetState;
  }
  
  public boolean atState() {
    boolean atPositionState = Units.Radians.of(getAngle().getRadians()).isNear(
        Units.Radians.of(m_targetState.getArmSetpoint().getRadians()), 
        CoralConstants.kCoralArmAtStatePositionTolerance
    );

    boolean atVelocityState = 
        m_coralArmEncoder.getVelocity() < 
        CoralConstants.kCoralArmAtStateVelocityTolerance.in(Units.Rotations.per(Units.Minute));

    return atVelocityState && atPositionState;
  }

  /* Overrides */

  @Override
  public void periodic() {
    super.periodic();
    double pidCalculatedVoltage = m_coralArmPIDController.calculate(getAngle().getRadians(), m_armDesiredAngle.getRadians());
    double feedforwardCalculatedVoltage = m_coralArmFeedforwardController.calculate(getAngle().getRadians(), 0);

    double calculatedVoltage = pidCalculatedVoltage + feedforwardCalculatedVoltage;
    calculatedVoltage = MathUtil.clamp(calculatedVoltage, -12, 12);

    // SmartDashboard.putNumber("Coral Arm PID Voltage", pidCalculatedVoltage);
    // SmartDashboard.putNumber("Coral Arm FF Voltage", feedforwardCalculatedVoltage);
    SmartDashboard.putNumber("Coral Arm Applied Voltage", calculatedVoltage);

    m_coralArmMotorController.setVoltage(calculatedVoltage);
  }

  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Coral Arm Desired Angle", m_armDesiredAngle.getDegrees());
    SmartDashboard.putNumber("Coral Arm Measured Angle", getAngle().getDegrees());
    // SmartDashboard.putNumber("Coral Arm Measured Degrees Raw", Rotation2d.fromRadians(m_coralArmEncoder.getPosition()).getDegrees());
  }

  protected void dashboardInit() {}
  protected void publishInit() {}
  protected void publishPeriodic() {}
}