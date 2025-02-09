// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.AlgaeConstants;
import frc.robot.subsystems.Subsystem;

public class AlgaeIntakeSubsystem extends Subsystem {
  private final SparkFlex m_AlgaeIntakeMotor;
  private final RelativeEncoder m_AlgaeIntakeEncoder;
  private final SparkClosedLoopController m_AlgaeIntakeClosedLoopController;

  private LinearVelocity m_currentVelocity;

  protected static AlgaeIntakeSubsystem m_instance;
  public static AlgaeIntakeSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new AlgaeIntakeSubsystem();
      return m_instance;
  }

  public AlgaeIntakeSubsystem() {
    super();

    m_AlgaeIntakeMotor = new SparkFlex(AlgaeConstants.CANIDs.kIntake, MotorType.kBrushless);
    m_AlgaeIntakeEncoder = m_AlgaeIntakeMotor.getEncoder();
    m_AlgaeIntakeClosedLoopController = m_AlgaeIntakeMotor.getClosedLoopController();

    SparkFlexConfig AlgaeIntakeConfig = new SparkFlexConfig();

    AlgaeIntakeConfig
      .smartCurrentLimit((int) AlgaeConstants.kArmCurrentLimit.in(Units.Amps));

    AlgaeIntakeConfig
      .encoder
      .positionConversionFactor(
        AlgaeConstants.ConversionFactors.Intake.kPositionConversionFactor.in(Units.Meters)
      )
      .velocityConversionFactor(
        AlgaeConstants.ConversionFactors.Intake.kVelocityConversionFactor.in(Units.MetersPerSecond)
      );
    
    AlgaeIntakeConfig
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(
        AlgaeConstants.PID.Intake.kP,
        AlgaeConstants.PID.Intake.kI,
        AlgaeConstants.PID.Intake.kD
      )
      .outputRange(-1, 1);

      m_AlgaeIntakeMotor.configure(AlgaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(LinearVelocity desiredVelocity) {     
    m_AlgaeIntakeClosedLoopController.setReference(desiredVelocity.in(Units.MetersPerSecond), ControlType.kVelocity);
  }

  private void setMotor(double speed) {
    m_AlgaeIntakeMotor.set(speed);
  }

  // Dashboard Fluff //
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {}

  protected void publishInit() {}
  protected void publishPeriodic() {}
}