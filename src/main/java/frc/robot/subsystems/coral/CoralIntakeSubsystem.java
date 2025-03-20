// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import java.lang.Thread.State;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
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

public class CoralIntakeSubsystem extends SubsystemAbstract {
  public enum CoralIntakeState {
    Rest,
    Outtake,
    Intake,
    Hold,
    AlgaeRemove,
    DebugEject
  }

  private final SparkFlex m_coralIntakeMotor;
  private TimeOfFlight m_tofSensor;

  private CoralIntakeState m_coralIntakeState = CoralIntakeState.Rest;

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

      m_tofSensor = new TimeOfFlight(CoralConstants.CANIDs.kCoralIntakeTimeOfFlight);
      m_tofSensor.setRangeOfInterest(
        8 - CoralConstants.kCoralIntakeTOFRangeOfInterest, 8 - CoralConstants.kCoralIntakeTOFRangeOfInterest, 
        8 + CoralConstants.kCoralIntakeTOFRangeOfInterest, 8 + CoralConstants.kCoralIntakeTOFRangeOfInterest
      );
  }

  /* Setters and Getters */
  public Distance getTOFReading() {
    return Units.Millimeters.of(m_tofSensor.getRange()).minus(CoralConstants.kTOFCorrection);
  }

  public boolean hasPiece() {
    return getTOFReading().in(Units.Millimeters) <= CoralConstants.kCoralMaxTOFReading.in(Units.Millimeters);
  }

  public Distance getPieceDispFromCenter() {
    if (!hasPiece())
      return Units.Meters.zero();
    // positive means shift right
    return getTOFReading().minus(CoralConstants.kCenteredCoralReading);
  }

  public void setVoltage(Voltage voltage) {
    double volts = voltage.in(Units.Volts);

    volts = MathUtil.clamp(volts, -12, 12);
    // System.out.println("test " + volts);

    m_coralIntakeMotor.setVoltage(volts);
  }

  public Current getCurrent() {
    return Units.Amps.of(m_coralIntakeMotor.getAppliedOutput());
  }

  public void setState(CoralIntakeState state) {
    m_coralIntakeState = state;

    switch (m_coralIntakeState) {
      case Rest:
        setVoltage(Units.Volts.zero());
        break;
      case Intake:
        setVoltage(CoralConstants.kCoralIntakeVoltage);
        break;
      case Outtake:
        setVoltage(CoralConstants.kCoralOuttakeVoltage);
        break;
      case Hold:
        setVoltage(CoralConstants.kCoralHoldVoltage);
        break;
      case AlgaeRemove:
        setVoltage(CoralConstants.kCoralAlgaeRemoveVoltage);
        break;
      case DebugEject:
        setVoltage(CoralConstants.kCoralDebugEjectVoltage);
        break;
      default:
        break;
    }
  }

  /* Overrides */

  @Override
  public void periodic() {
    super.periodic();
    
    if (
      hasPiece() && (m_coralIntakeState == CoralIntakeState.Rest) && 
      (m_coralIntakeState != CoralIntakeState.DebugEject)
    ) {
      setState(CoralIntakeState.Hold);
    }
  }

  protected void dashboardInit() {}

  protected void dashboardPeriodic() {
    // SmartDashboard.putNumber("Coral Intake Current", getCurrent().in(Units.Amps));

    SmartDashboard.putBoolean("Coral Intake TOF Has Piece", hasPiece());
    // SmartDashboard.putNumber("Coral Intake TOF Center Disp Inches", getPieceDispFromCenter().in(Units.Inches));
  }

  protected void publishInit() {}
  protected void publishPeriodic() {}
}
