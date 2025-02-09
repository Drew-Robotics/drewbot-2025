// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.LinearVelocity;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.constants.AlgaeConstants;
import frc.robot.subsystems.Subsystem;

public class AlgaeIntakeSubsystem extends Subsystem {
  private final SparkMax m_algaeIntakeMotorController;
  private final RelativeEncoder m_algaeIntakeEncoder;

  private LinearVelocity m_currentVelocity;

  public AlgaeIntakeSubsystem() {
    super();

    m_algaeIntakeMotorController = new SparkMax(AlgaeConstants.CANIDs.kIntake, MotorType.kBrushless);
    m_algaeIntakeEncoder = m_algaeIntakeMotorController.getEncoder();

    
  }

  public void setVelocity(LinearVelocity velocity) {
    m_currentVelocity = velocity;
  }

  public void stop() {
    m_currentVelocity = m_currentVelocity.times(0);
  }

  private void setMotor(double speed) {
    m_algaeIntakeMotorController.set(speed);
  }

  @Override
  public void periodic() {
    super.periodic();

  }

  // Dashboard Fluff //
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {}

  protected void publishInit() {}
  protected void publishPeriodic() {}
}
