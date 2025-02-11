// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

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

import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class CoralIntakeSubsystem extends SubsystemAbstract implements CoralSubsystemI {
  private final SparkFlex m_coralIntakeMotor;
  private final RelativeEncoder m_coralIntakeEncoder;
  private final SparkClosedLoopController m_coralIntakeClosedLoopController;

  private LinearVelocity m_currentVelocity;

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
    m_coralIntakeClosedLoopController = m_coralIntakeMotor.getClosedLoopController();

    SparkFlexConfig coralIntakeConfig = new SparkFlexConfig();

    coralIntakeConfig
      .smartCurrentLimit((int) CoralConstants.kArmCurrentLimit.in(Units.Amps));
 
    coralIntakeConfig
      .encoder
      .positionConversionFactor(
        CoralConstants.ConversionFactors.Intake.kPositionConversionFactor.in(Units.Meters)
      )
      .velocityConversionFactor(
        CoralConstants.ConversionFactors.Intake.kVelocityConversionFactor.in(Units.MetersPerSecond)
      );
    
    coralIntakeConfig
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(
        CoralConstants.PID.CoralIntake.kP,
        CoralConstants.PID.CoralIntake.kI,
        CoralConstants.PID.CoralIntake.kD
      )
      .outputRange(-1, 1);

      m_coralIntakeMotor.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /* Setters and Getters */
  public void setVelocity(LinearVelocity desiredVelocity) {
    m_currentVelocity = desiredVelocity;
    m_coralIntakeClosedLoopController.setReference(m_currentVelocity.in(Units.MetersPerSecond), ControlType.kVelocity);
  }

  public void setState(CoralState state) {
    if (state.getIntakeRunning()){
      setVelocity(CoralConstants.kIntakeSurfaceVelocity);
    }
  }

  /* Overrides */
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {}

  protected void publishInit() {}
  protected void publishPeriodic() {}
}
