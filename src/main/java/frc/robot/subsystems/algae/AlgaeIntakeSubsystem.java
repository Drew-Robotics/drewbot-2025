// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class AlgaeIntakeSubsystem extends SubsystemAbstract {
  private final SparkFlex m_algaeIntakeMotor;
  private final RelativeEncoder m_algaeIntakeEncoder;
  private final SparkClosedLoopController m_algaeIntakeClosedLoopController;

  private LinearVelocity m_desiredVelocity = MetersPerSecond.zero();

  protected static AlgaeIntakeSubsystem m_instance;
  public static AlgaeIntakeSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new AlgaeIntakeSubsystem();
      return m_instance;
  }

  public AlgaeIntakeSubsystem() {
    super();

    m_algaeIntakeMotor = new SparkFlex(AlgaeConstants.CANIDs.kIntake, MotorType.kBrushless);
    m_algaeIntakeEncoder = m_algaeIntakeMotor.getEncoder();
    m_algaeIntakeClosedLoopController = m_algaeIntakeMotor.getClosedLoopController();

    SparkFlexConfig algaeIntakeConfig = new SparkFlexConfig();

    algaeIntakeConfig
      .smartCurrentLimit((int) AlgaeConstants.kArmCurrentLimit.in(Units.Amps))
      .inverted(AlgaeConstants.kAlgaeIntakeMotorInverted)
      .idleMode(AlgaeConstants.IdleModes.kIntake);

    // algaeIntakeConfig
    //   .encoder
    //   .positionConversionFactor(
    //     AlgaeConstants.ConversionFactors.Intake.kPositionConversionFactor.in(Units.Meters)
    //   )
    //   .velocityConversionFactor(
    //     AlgaeConstants.ConversionFactors.Intake.kVelocityConversionFactor.in(Units.MetersPerSecond)
    //   );
    
    // algaeIntakeConfig
    //   .closedLoop
    //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //   .pid(
    //     AlgaeConstants.PID.Intake.kP,
    //     AlgaeConstants.PID.Intake.kI,
    //     AlgaeConstants.PID.Intake.kD
    //   )
    //   .outputRange(-1, 1);

      m_algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /* Getters and Setters */

  public void setVoltage(Voltage voltage) {
    double volts = voltage.in(Units.Volts);

    volts = MathUtil.clamp(volts, -12, 12);

    m_algaeIntakeMotor.setVoltage(volts);
  }

  public Current getCurrent() {
    return Units.Amps.of(m_algaeIntakeMotor.getOutputCurrent());
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(m_algaeIntakeEncoder.getVelocity());
  }

  /* Overrides */
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Algae Intake Current", getCurrent().in(Units.Amps));
  }

  protected void publishInit() {}
  protected void publishPeriodic() {}
}