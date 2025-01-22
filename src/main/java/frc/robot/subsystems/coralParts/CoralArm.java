// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralParts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.PID;
import frc.robot.subsystems.Subsystem;

public class CoralArm extends Subsystem {
  private final SparkPIDController m_armPID;
  private Rotation2d m_armMeasuredAngle;
  private Rotation2d m_armDesiredAngle;

  private CANSparkMax m_armMotorController;
  private SparkAbsoluteEncoder m_armEncoder;


  /** Creates a new Coralarm. */
  public CoralArm() {
    super(CoralArm.class.getSimpleName());

    m_armMotorController = new CANSparkMax(CANIDs.coralArm, MotorType.kBrushless);


    m_armEncoder = m_armMotorController.getAbsoluteEncoder();
    m_armPID = m_armMotorController.getPIDController();

    m_armPID.setP(PID.CoralArm.kP);
    m_armPID.setI(PID.CoralArm.kI);
    m_armPID.setD(PID.CoralArm.kD);

    m_armPID.setOutputRange();
    
    m_armPID.setFeedbackDevice(m_armEncoder);
  }

  @Override
  public void periodic() {
    m_armMeasuredAngle = Rotation2d.fromRotations(m_armEncoder.getPosition());
    
    updatePIDController();
  }

  private void updatePIDController() {
    m_armPID.setReference(m_armDesiredAngle.getRadians(), ControlType.kPosition);
  }

  public void setDesiredAngle(Rotation2d angle) {
    m_armDesiredAngle = angle;
  }

  @Override
  protected void dashboardPeriodic() {
    SmartDashboard.putNumber("Desired Arm Angle (degrees)", m_armDesiredAngle.getDegrees());
    SmartDashboard.putNumber("Measured Arm Angle (degrees)", m_armMeasuredAngle.getDegrees());
  }
}
