// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class AlgaeIntakeSubsystem extends SubsystemAbstract {
  // public enum AlgaeIntakeState {
  //   Rest,
  //   Outtake,
  //   Intake,
  //   Hold
  // }

  private final SparkFlex m_algaeIntakeMotor;
  private TimeOfFlight m_tofSensor;
  private double m_hasPieceTimer = Timer.getFPGATimestamp();

  // private AlgaeIntakeState m_algaeIntakeState = AlgaeIntakeState.Rest;

  protected static AlgaeIntakeSubsystem m_instance;
  public static AlgaeIntakeSubsystem getInstance() {
      if (m_instance == null)
          m_instance = new AlgaeIntakeSubsystem();
      return m_instance;
  }

  public AlgaeIntakeSubsystem() {
    super();

    m_algaeIntakeMotor = new SparkFlex(AlgaeConstants.CANIDs.kIntake, MotorType.kBrushless);

    SparkFlexConfig algaeIntakeConfig = new SparkFlexConfig();

    algaeIntakeConfig
      .smartCurrentLimit((int) AlgaeConstants.kArmCurrentLimit.in(Units.Amps))
      .inverted(AlgaeConstants.kAlgaeIntakeMotorInverted)
      .idleMode(AlgaeConstants.IdleModes.kIntake);

    m_algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_tofSensor = new TimeOfFlight(AlgaeConstants.CANIDs.kSensor);
  }

  @Override
  public void periodic() {
    super.periodic();
    hasPieceRaw();
  }

  /* Getters and Setters */

  public Distance getTOFReading() {
    return Units.Millimeters.of(m_tofSensor.getRange());
  }

  public boolean hasPieceRaw() {
    boolean hasPieceRawBool = getTOFReading().in(Units.Millimeters) <= AlgaeConstants.kAlgaeMaxTOFReading.in(Units.Millimeters) 
      && subsystems.algaeArm.getAngle().getDegrees() <= AlgaeConstants.kMaxRestAngle.getDegrees();

    if (hasPieceRawBool && m_hasPieceTimer == Double.MAX_VALUE) {
      m_hasPieceTimer = Timer.getFPGATimestamp();
    } else if (!hasPieceRawBool){
      m_hasPieceTimer = Double.MAX_VALUE;
    }

    return hasPieceRawBool;
  }

  public boolean hasPiece() {
    return Timer.getFPGATimestamp() - m_hasPieceTimer  > AlgaeConstants.kAlgaeTimerSeconds;
  }

  public void setVoltage(Voltage voltage) {
    double volts = voltage.in(Units.Volts);

    volts = MathUtil.clamp(volts, -12, 12);

    m_algaeIntakeMotor.setVoltage(volts);
  }

  public Current getCurrent() {
    return Units.Amps.of(m_algaeIntakeMotor.getOutputCurrent());
  }


  /* Overrides */
  protected void dashboardInit() {}

  protected void dashboardPeriodic() {
    // SmartDashboard.putNumber("Algae Intake Current", getCurrent().in(Units.Amps));
    SmartDashboard.putBoolean("Algae Intake TOF Has Piece", hasPiece());
    SmartDashboard.putBoolean("Algae Intake TOF Has Piece", hasPieceRaw());
    SmartDashboard.putNumber("Algae Intake TOF Has Piece Timer", m_hasPieceTimer);
    SmartDashboard.putNumber("Algae TOF Reading Inches", getTOFReading().in(Inches));
  }

  protected void publishInit() {}
  protected void publishPeriodic() {}
}