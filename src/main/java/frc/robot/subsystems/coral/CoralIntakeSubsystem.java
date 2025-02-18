// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class CoralIntakeSubsystem extends SubsystemAbstract implements CoralSubsystemI {
  public enum CoralIntakeStates {
    Rest,
    Outtake,
    Intake
  }

  private final SparkFlex m_coralIntakeMotor;
  private final RelativeEncoder m_coralIntakeEncoder;
  // private final SparkClosedLoopController m_coralIntakeClosedLoopController;

  // private LinearVelocity m_desiredVelocity = MetersPerSecond.zero();

  protected static CoralIntakeSubsystem m_instance;
  public static CoralIntakeSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new CoralIntakeSubsystem();
      return m_instance;
  }

  public CoralIntakeSubsystem() {
    super();

    m_coralIntakeMotor = new SparkFlex(CoralConstants.CANIDs.kCoralIntake, MotorType.kBrushless);
    m_coralIntakeEncoder = m_coralIntakeMotor.getEncoder();
    // m_coralIntakeClosedLoopController = m_coralIntakeMotor.getClosedLoopController();

    SparkFlexConfig coralIntakeConfig = new SparkFlexConfig();

    coralIntakeConfig
      .smartCurrentLimit((int) CoralConstants.kArmCurrentLimit.in(Units.Amps))
      .inverted(CoralConstants.kCoralIntakeMotorInverted)
      .idleMode(CoralConstants.IdleModes.kIntake);
 
    // coralIntakeConfig
    //   .encoder
    //   .positionConversionFactor(
    //     CoralConstants.ConversionFactors.Intake.kPositionConversionFactor.in(Units.Meters)
    //   )
    //   .velocityConversionFactor(
    //     CoralConstants.ConversionFactors.Intake.kVelocityConversionFactor.in(Units.MetersPerSecond)
    //   );
    
    // coralIntakeConfig
    //   .closedLoop
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   .pid(
    //     CoralConstants.PID.CoralIntake.kP,
    //     CoralConstants.PID.CoralIntake.kI,
    //     CoralConstants.PID.CoralIntake.kD
    //   )
    //   .outputRange(-1, 1);

      m_coralIntakeMotor.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* Setters and Getters */

  public LinearVelocity getVelocity() {
    return Units.MetersPerSecond.of(m_coralIntakeEncoder.getVelocity());
  }

  public void setVoltage(Voltage voltage) {
    double volts = voltage.in(Units.Volts);

    volts = MathUtil.clamp(volts, -12, 12);

    m_coralIntakeMotor.setVoltage(volts);
  }

  public Current getCurrent() {
    return Units.Amps.of(m_coralIntakeMotor.getAppliedOutput());
  }

  public void setState(CoralState state) {
    switch (state.getIntakeState()) {
      case Rest:
        setVoltage(Units.Volts.zero());
        break;
      case Intake:
        setVoltage(CoralConstants.kIntakeVoltage);
        break;
      case Outtake:
        setVoltage(CoralConstants.kOuttakeVoltage);
        break;
      default:
        break;
    }
  }

  /* Overrides */
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Coral Intake Current (Amps)", getCurrent().in(Units.Amps));
  }

  protected void publishInit() {}
  protected void publishPeriodic() {}
}
