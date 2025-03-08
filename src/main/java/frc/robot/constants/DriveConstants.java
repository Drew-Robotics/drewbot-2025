package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
  public static final Distance kWheelBase = Units.Inches.of(23);
  public static final Distance kTrackWidth = Units.Inches.of(23);

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
    
    public static final int kDrivingMotorPinionTeeth = 12;
    public static final int kSpurGearTeeth = 22;

    // 45 teeth on the wheels bevel gear and 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (double) (45d * kSpurGearTeeth) / (double) (kDrivingMotorPinionTeeth * 15d);

    public static final LinearVelocity kDriveWheelFreeSpeed = Units.MetersPerSecond.of(
      kDriveMotorFreeSpeed.in(Units.RevolutionsPerSecond) * kWheelCircumference.in(Units.Meters) / kDrivingMotorReduction
    );
  }

  public static final class EncoderConversions {
    public static final Distance kDrivingEncoderPositionFactor = SwerveCalculations.kWheelCircumference.div(SwerveCalculations.kDrivingMotorReduction);
    public static final LinearVelocity kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor.per(Units.Minute);

    public static final Angle kTurningEncoderPositionFactor = Units.Radians.of(2 * Math.PI);
    public static final AngularVelocity kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor.per(Units.Minute); // TODO : wtf ???
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
    public static final int kFrontLeftDriving = 5;
    public static final int kBackLeftDriving = 3;
    public static final int kFrontRightDriving = 7;
    public static final int kBackRightDriving = 1;

    public static final int kFrontLeftTurning = 6;
    public static final int kBackLeftTurning = 4;
    public static final int kFrontRightTurning = 8;
    public static final int kBackRightTurning = 2;
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
    public static final boolean kFrontLeft = true;
    public static final boolean kFrontRight = true;
    public static final boolean kBackLeft = true;
    public static final boolean kBackRight = true;
  }
  
  public static final class TurningInverted {
    public static final boolean kFrontLeft = false;
    public static final boolean kFrontRight = false;
    public static final boolean kBackLeft = false;
    public static final boolean kBackRight = false;
  }

  public static final class DrivingMotorPID {
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 1 / SwerveCalculations.kDriveWheelFreeSpeed.in(MetersPerSecond);
  }

  public static final class TurningPID {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  // for stuff like auto rotate and turn to angle

  // this is for teleop non pathplanner
  public static final class DrivingPID {
    public static final double kP = 40; // Meters -> Meters per second
    public static final double kI = 5;
    public static final double kD = 0;
    public static final LinearVelocity kMaxVel = MetersPerSecond.of(0.3);
    public static final LinearAcceleration kMaxAccel = MetersPerSecondPerSecond.of(3);
  }

  public static final class RotationPID {
    public static final double kP = 7;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final AngularVelocity kMaxVel = DegreesPerSecond.of(120);
    public static final AngularAcceleration kMaxAccel = DegreesPerSecondPerSecond.of(500);
  }

  public static final Distance kPositionTolerance = Units.Inches.of(2);
  public static final Angle kRotationTolerance = Units.Degrees.of(2);

  public static final Current kDrivingMotorCurrentLimit = Amps.of(40);
  public static final Current kTurningMotorCurrentLimit = Amps.of(20);
}
