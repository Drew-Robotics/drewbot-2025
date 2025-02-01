package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

import frc.robot.constants.DriveConstants;

public class SwerveModule {

  private SwerveModuleState m_measuredState = new SwerveModuleState();
  private SwerveModuleState m_commandedState = new SwerveModuleState();
  private SwerveModulePosition m_measuredPosition = new SwerveModulePosition();

  private final SparkFlex m_driveMotor;
  public final SparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

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

    m_commandedPreOpStateTopic = m_moduleTable.getStructTopic(moduleName + " Commanded State Pre Optimization", SwerveModuleState.struct);
    m_commandedPreOpStatePublisher = m_commandedPreOpStateTopic.publish();

    m_commandedStateTopic = m_moduleTable.getStructTopic(moduleName + " Commanded State", SwerveModuleState.struct);
    m_commandedStatePublisher = m_commandedStateTopic.publish();

    m_measuredPositionTopic = m_moduleTable.getStructTopic(moduleName + " Measured Position", SwerveModulePosition.struct);
    m_measuredPositionPublisher = m_measuredPositionTopic.publish();

    m_driveMotor = new SparkFlex(driveMotorCanID, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorCanID, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_drivingEncoder = m_driveMotor.getEncoder();
    m_drivingPID = m_driveMotor.getPIDController();
    m_drivingPID.setFeedbackDevice(m_drivingEncoder);
    m_driveMotor.setIdleMode(IdleMode.kBrake);

    // turning motor uses absolute encoder
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPID = m_turningMotor.getPIDController();
    m_turningPID.setFeedbackDevice(m_turningEncoder);

    m_turningPID.setPositionPIDWrappingEnabled(true);
    m_turningPID.setPositionPIDWrappingMinInput(0);
    m_turningPID.setPositionPIDWrappingMaxInput(2 * Math.PI);

    m_turningMotor.setIdleMode(IdleMode.kCoast);

    // go from rotations or rotations per minute to meters or meters per second
    m_drivingEncoder.setPositionConversionFactor(DriveConstants.SwerveCalculations.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DriveConstants.SwerveCalculations.kDrivingEncoderPositionFactor / 60);

    // go from rotations or rotations per minute to meters or meters per second
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI);
    m_turningEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

    // turning motor must be inverted, output shaft rotates in the opposite direction of turning
    m_turningEncoder.setInverted(true);

    // set up pid controllers
    m_drivingPID.setP(DriveConstants.DrivingPID.kP);
    m_drivingPID.setI(DriveConstants.DrivingPID.kI);
    m_drivingPID.setD(DriveConstants.DrivingPID.kD);
    m_drivingPID.setFF(DriveConstants.DrivingPID.kFF);
    m_drivingPID.setOutputRange(-1, 1);

    m_turningPID.setP(DriveConstants.TurningPID.kP);
    m_turningPID.setI(DriveConstants.TurningPID.kI);
    m_turningPID.setD(DriveConstants.TurningPID.kD);
    m_turningPID.setFF(DriveConstants.TurningPID.kFF);
    m_turningPID.setOutputRange(-1, 1);

    // current limits
    m_driveMotor.setSmartCurrentLimit((int) DriveConstants.kDrivingMotorCurrentLimit.in(Units.Volts));
    m_turningMotor.setSmartCurrentLimit((int) DriveConstants.kTurningMotorCurrentLimit.in(Units.Volts));

    m_driveMotor.setInverted(drivingMotorInverted);
    m_turningMotor.setInverted(turningMotorInverted);

    m_driveMotor.burnFlash();
    m_turningMotor.burnFlash();
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
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedState, new Rotation2d(m_turningEncoder.getPosition()));
    m_commandedState = optimizedDesiredState;

    m_drivingPID.setReference(m_commandedState.speedMetersPerSecond, SparkFlex.ControlType.kVelocity);
    m_turningPID.setReference(m_commandedState.angle.getRadians(), SparkMax.ControlType.kPosition);

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
