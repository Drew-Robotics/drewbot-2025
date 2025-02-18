package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.units.Units;
import frc.robot.constants.DriveConstants;

public class SwerveModule {

  private SwerveModuleState m_measuredState = new SwerveModuleState();
  private SwerveModuleState m_commandedState = new SwerveModuleState();
  
  private SwerveModulePosition m_measuredPosition = new SwerveModulePosition();

  private final SparkFlex m_drivingMotor;
  public final SparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private final String m_moduleName;
  private final Rotation2d m_angularOffset;

  private final NetworkTable m_moduleTable;

  private final StructTopic<SwerveModuleState> m_measuredStateTopic;
  private final StructPublisher<SwerveModuleState> m_measuredStatePublisher;

  private final StructTopic<SwerveModuleState> m_commandedPreOpStateTopic;
  private final StructPublisher<SwerveModuleState> m_commandedPreOpStatePublisher;

  private final StructTopic<SwerveModuleState> m_commandedStateTopic;
  private final StructPublisher<SwerveModuleState> m_commandedStatePublisher;

  private final StructTopic<SwerveModulePosition> m_measuredPositionTopic;
  private final StructPublisher<SwerveModulePosition> m_measuredPositionPublisher;

  /**
   * Swerve Module Constructor.
   * 
   * @param driveMotorCanID Can ID of the driving Motor
   * @param turningMotorCanID Can ID of the turning motor
   * @param moduleName The name of the module. Ex: Front Right, Back Left 
   * @param angularOffset The angle offset of the module
   * @param superTable The network table for the module to publish its table under
   */
  public SwerveModule(String moduleName, int driveMotorCanID, int turningMotorCanID, Rotation2d angularOffset, NetworkTable superTable, boolean drivingMotorInverted, boolean turningMotorInverted) {
    m_moduleName = moduleName;

    m_angularOffset = angularOffset;

    m_moduleTable = superTable.getSubTable(m_moduleName);

    m_measuredStateTopic = m_moduleTable.getStructTopic(moduleName + " Measured State", SwerveModuleState.struct);
    m_measuredStatePublisher = m_measuredStateTopic.publish();

    m_commandedPreOpStateTopic = m_moduleTable.getStructTopic(moduleName + " Commanded State Pre-Optimization", SwerveModuleState.struct);
    m_commandedPreOpStatePublisher = m_commandedPreOpStateTopic.publish();

    m_commandedStateTopic = m_moduleTable.getStructTopic(moduleName + " Commanded State", SwerveModuleState.struct);
    m_commandedStatePublisher = m_commandedStateTopic.publish();

    m_measuredPositionTopic = m_moduleTable.getStructTopic(moduleName + " Measured Position", SwerveModulePosition.struct);
    m_measuredPositionPublisher = m_measuredPositionTopic.publish();

    m_drivingMotor = new SparkFlex(driveMotorCanID, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorCanID, MotorType.kBrushless);

    SparkFlexConfig drivingConfig = new SparkFlexConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();

    drivingConfig
      .inverted(drivingMotorInverted)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit((int) DriveConstants.kDrivingMotorCurrentLimit.in(Units.Amps));
    drivingConfig.encoder
      .positionConversionFactor(DriveConstants.EncoderConversions.kDrivingEncoderPositionFactor.in(Units.Meters))
      .velocityConversionFactor(DriveConstants.EncoderConversions.kDrivingEncoderVelocityFactor.in(Units.MetersPerSecond));
    drivingConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(
        DriveConstants.DrivingPID.kP, 
        DriveConstants.DrivingPID.kI, 
        DriveConstants.DrivingPID.kD, 
        DriveConstants.DrivingPID.kFF
      )
      .outputRange(-1,1);

    turningConfig
      .inverted(turningMotorInverted)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit((int) DriveConstants.kTurningMotorCurrentLimit.in(Units.Amps));
    turningConfig.absoluteEncoder
      .inverted(true)
      .positionConversionFactor(DriveConstants.EncoderConversions.kTurningEncoderPositionFactor.in(Units.Radians))
      .velocityConversionFactor(DriveConstants.EncoderConversions.kTurningEncoderVelocityFactor.in(Units.RadiansPerSecond));
    turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(
        DriveConstants.TurningPID.kP, 
        DriveConstants.TurningPID.kI, 
        DriveConstants.TurningPID.kD
      )
      .outputRange(-1,1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, DriveConstants.EncoderConversions.kTurningEncoderPositionFactor.in(Units.Radians));

    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingMotor.getClosedLoopController();
    m_turningClosedLoopController = m_turningMotor.getClosedLoopController();

    // Comment from https://github.com/REVrobotics/2025-REV-ION-FRC-Starter-Bot/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java#L53
    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingMotor.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Get the the state of the module. Encodes driving velocity and turning angle.
   * 
   * @return Module State
   */
  public SwerveModuleState getModuleState() {
    SwerveModuleState moduleState = new SwerveModuleState(
      Units.MetersPerSecond.of(m_drivingEncoder.getVelocity()),
      getCurrentAngle()
    );

    m_measuredState = moduleState;

    m_measuredStatePublisher.set(m_measuredState);
    return m_measuredState;
  }

  public Rotation2d getCurrentAngle() {
    return new Rotation2d(m_turningEncoder.getPosition()).minus(m_angularOffset); // module relative -> robot relative
  }

  /**
   * Command a module state. Param encodes driving velocity and turning angle.
   * 
   * @param moduleState Module State
   */
  public void setModuleState(SwerveModuleState moduleState) {

    SwerveModuleState correctedState = new SwerveModuleState(
      moduleState.speedMetersPerSecond,
      moduleState.angle.plus(m_angularOffset) // robot relative -> module relative
    );

    m_commandedPreOpStatePublisher.set(correctedState);

    // avoid spining more than 90 degrees
    // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedState, m_commandedState.angle.minus(m_angularOffset)); // module relative -> robot relative
    correctedState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    m_commandedState = correctedState;

    m_drivingClosedLoopController.setReference(m_commandedState.speedMetersPerSecond, SparkFlex.ControlType.kVelocity);
    m_turningClosedLoopController.setReference(m_commandedState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_commandedStatePublisher.set(m_commandedState);
  }

  /**
   * Gets the modules position object. Encodes driving distance and turning angle.
   * 
   * @return Module Position
   */
  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition modulePosition = new SwerveModulePosition(
        Units.Meters.of(m_drivingEncoder.getPosition()),
        getCurrentAngle()
    );

    m_measuredPosition = modulePosition;

    m_measuredPositionPublisher.set(m_measuredPosition);
    return m_measuredPosition;
  }
}
