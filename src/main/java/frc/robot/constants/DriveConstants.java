package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
  public static final Distance kWheelBase = Units.Inches.of(24.5);
  public static final Distance kTrackWidth = Units.Inches.of(24.5);

  public static final Boolean kGyroReversed = true;
  public static final Boolean kFieldOriented = true;

  public static final NavXUpdateRate kUpdateRate = NavXUpdateRate.k100Hz;

  public static final class SwerveCalculations {
    public static final DCMotor kDriveMotor = DCMotor.getNeoVortex(1);
    public static final DCMotor kTurningMotor = DCMotor.getNeo550(1);

    public static final double kWheelCoefficientOfFriction = 1.45;
    public static final AngularVelocity kDriveMotorFreeSpeed = Units.RadiansPerSecond.of(kDriveMotor.freeSpeedRadPerSec);

    public static final Distance kWheelDiameter = Units.Meters.of(0.0762);
    public static final Distance kWheelCircumference = Units.Meters.of(kWheelDiameter.in(Units.Meters) * Math.PI);
    
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final int kSpurGearTeeth = 20;

    // 45 teeth on the wheels bevel gear and 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (double) (45d * kSpurGearTeeth) / (double) (kDrivingMotorPinionTeeth * 15d);

    public static final Velocity kDriveWheelFreeSpeed = Units.MetersPerSecond(kDriveMotorFreeSpeed.in(Units.RevolutionsPerSecond) * kWheelCircumference.in(Units.Meters)) / kDrivingMotorReduction;
    // convert from rotations to meters
    public static final double kDrivingEncoderPositionFactor = kWheelCircumference.in(Units.Meters) / kDrivingMotorReduction;
  }
  
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase.div(2), kTrackWidth.div(2)),
    new Translation2d(kWheelBase.div(2), kTrackWidth.div(2).negate()),
    new Translation2d(kWheelBase.div(2).negate(), kTrackWidth.div(2)),
    new Translation2d(kWheelBase.div(2).negate(), kTrackWidth.div(2).negate())
  );

  public static final class AngularOffsets {
    public static final Rotation2d kFrontLeft = Rotation2d.fromDegrees(-90);
    public static final Rotation2d kFrontRight = Rotation2d.fromDegrees(0);
    public static final Rotation2d kBackLeft = Rotation2d.fromDegrees(180);
    public static final Rotation2d kBackRight = Rotation2d.fromDegrees(90);
  }


  public static final class DriveCANIDs {
    public static final int kFrontLeftDriving = 1;
    public static final int kBackLeftDriving = 5;
    public static final int kFrontRightDriving = 3;
    public static final int kBackRightDriving = 7;

    public static final int kFrontLeftTurning = 2;
    public static final int kBackLeftTurning = 6;
    public static final int kFrontRightTurning = 4;
    public static final int kBackRightTurning = 8;
  }

  public static final class MaxVels {
    public static final LinearVelocity kTranslationalVelocity = MetersPerSecond.of(4.5);
    public static final AngularVelocity kRotationalVelocity = RadiansPerSecond.of(2 * Math.PI);
  }

  public static final class SlewRate {
    public static final double kMag = 2;
    public static final double kDir = 2;
    public static final double kRot = 2;
  }

  public static final class DrivingInverted {
    public static final boolean kFrontLeft = false;
    public static final boolean kFrontRight = false;
    public static final boolean kBackLeft = false;
    public static final boolean kBackRight = false;
  }
  
  public static final class TurningInverted {
    public static final boolean kFrontLeft = false;
    public static final boolean kFrontRight = false;
    public static final boolean kBackLeft = false;
    public static final boolean kBackRight = false;
  }

  public static final class DrivingPID {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 1 / SwerveCalculations.kDriveWheelFreeSpeed;
  }

  public static final class TurningPID {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
  }

  public static final Current kDrivingMotorCurrentLimit = Amps.of(50);
  public static final Current kTurningMotorCurrentLimit = Amps.of(20);

  // go from rotations or rotations per minute to meters or meters per second
  public static final double kDrivingEncoderPositionFactor = SwerveCalculations.kDrivingEncoderPositionFactor;
}
