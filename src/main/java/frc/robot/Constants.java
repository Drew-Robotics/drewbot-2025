// package frc.robot;

// import java.util.List;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.units.measure.measure.*;

// public class Constants {

//   public static final class DriveConstants {

//     public static final Distance kWheelBase = Units.Inches.of(24.5);
//     public static final Distance kTrackWidth = Units.Inches.of(24.5);

//     public static final Boolean kGyroReversed = true;

//     public static final Boolean kFieldOriented = true;

//     public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
//       new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2)),
//       new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2).negate()),
//       new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2)),
//       new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2).negate())
//     );

//     public static final class CanIDs {
//       public static final int kFrontLeftDriving = 1;
//       public static final int kBackLeftDriving = 5;
//       public static final int kFrontRightDriving = 3;
//       public static final int kBackRightDriving = 7;

//       public static final int kFrontLeftTurning = 2;
//       public static final int kBackLeftTurning = 6;
//       public static final int kFrontRightTurning = 4;
//       public static final int kBackRightTurning = 8;
//     }

//     public static final class AngularOffsets {
//       public static final Rotation2d kFrontLeft = new Rotation2d(-Math.PI / 2);
//       public static final Rotation2d kFrontRight = new Rotation2d(0);
//       public static final Rotation2d kBackLeft = new Rotation2d(Math.PI);
//       public static final Rotation2d kBackRight = new Rotation2d(Math.PI / 2);
//     }

//     public static final class MaxVels {
//       public static final LinearVelocity kTranslationalVelocity = Units.MetersPerSecond.of(4.5);
//       public static final AngularVelocity kRotationalVelocity = Units.RadiansPerSecond.of(2 * Math.PI);
//     }

//     public static final class SlewRate {
//       public static final double kMag = 2;
//       public static final double kDir = 2;
//       public static final double kRot = 2;
//     }
//   }

  

//   public static final class SwerveConstants {

//     public static final class SwerveCalculations {
//       public static final double kNeoFreeSpeedRpm = 5676d;
//       public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60d;

//       public static final double kWheelDiameterMeters = 0.0762;
//       public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      
//       public static final int kDrivingMotorPinionTeeth = 14;
//       public static final int kSpurGearTeeth = 20;

//       // 45 teeth on the wheels bevel gear and 15 teeth on the bevel pinion
//       public static final double kDrivingMotorReduction = (double) (45d * kSpurGearTeeth) / (double) (kDrivingMotorPinionTeeth * 15d);

//       public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
//       // convert from rotations to meters
//       public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDrivingMotorReduction;
//     }

//     public static final class DrivingInverted {
//       public static final boolean kFrontLeft = false;
//       public static final boolean kFrontRight = false;
//       public static final boolean kBackLeft = false;
//       public static final boolean kBackRight = false;
//     }
    
//     public static final class TurningInverted {
//       public static final boolean kFrontLeft = false;
//       public static final boolean kFrontRight = false;
//       public static final boolean kBackLeft = false;
//       public static final boolean kBackRight = false;
//     }

//     public static final class DrivingPID {
//       public static final double kP = 0.05;
//       public static final double kI = 0;
//       public static final double kD = 0;
//       public static final double kFF = 1 / SwerveCalculations.kDriveWheelFreeSpeedRps;
//     }
//     public static final class TurningPID {
//       public static final double kP = 1;
//       public static final double kI = 0;
//       public static final double kD = 0;
//       public static final double kFF = 0;
//     }

//     public static final Voltage kDrivingMotorCurrentLimit = Units.Volts.of(50);
//     public static final Voltage kTurningMotorCurrentLimit = Units.Volts.of(20);

//     // go from rotations or rotations per minute to meters or meters per second
//     public static final double kDrivingEncoderPositionFactor = SwerveCalculations.kDrivingEncoderPositionFactor;
//   }

//   public static final class AutoConstants {
//     public static final LinearVelocity kMaxSpeed = Units.MetersPerSecond.of(4.5);

//     public static final class DrivingPID {
//       public static final double kP = 0;
//       public static final double kI = 0;
//       public static final double kD = 0;
//     }
//     public static final class TurningPID {
//       public static final double kP = 0;
//       public static final double kI = 0;
//       public static final double kD = 0;
//     }
//   }

//   public static final class VisionConstants {

//     public static final class CameraNames {
//       public static final String kFrontLeft = "frontLeft";
//       public static final String kFrontRight = "frontRight";
//       public static final String kBackLeft = "backLeft";
//       public static final String kBackRight = "backRight";
//     }
//     public static final class CameraTransforms {
//       // Measure from the center line to the camera, not the 3D center of the robot.
//       public static final Distance lengthWidth = Units.Inches.of(14);
//       public static final Distance height = Units.Inches.of(8);
//       public static final Angle pitch = Units.Degrees.of(-28.125);
//       public static final Angle yawOffset = Units.Degrees.of(30);

//       public static final Transform3d kFrontLeft = new Transform3d(
//         -lengthWidth.in(Units.Meters),
//         -lengthWidth.in(Units.Meters),
//         height.in(Units.Meters), 
//         new Rotation3d(0, pitch.in(Units.Radians), - Math.PI + yawOffset.in(Units.Radians))
//       );
//       public static final Transform3d kFrontRight = new Transform3d(
//         -lengthWidth.in(Units.Meters),
//         lengthWidth.in(Units.Meters),
//         height.in(Units.Meters), 
//         new Rotation3d(0, pitch.in(Units.Radians), Math.PI - yawOffset.in(Units.Radians))
//       );
//       public static final Transform3d kBackLeft = new Transform3d(
//         lengthWidth.in(Units.Meters),
//         -lengthWidth.in(Units.Meters),
//         height.in(Units.Meters), 
//         new Rotation3d(0, pitch.in(Units.Radians), - yawOffset.in(Units.Radians))
//       );
//       public static final Transform3d kBackRight = new Transform3d(
//         lengthWidth.in(Units.Meters),
//         lengthWidth.in(Units.Meters),
//         height.in(Units.Meters), 
//         new Rotation3d(0, pitch.in(Units.Radians), yawOffset.in(Units.Radians))
//       );
//     }
    
//     public static final class AprilTags {
//     public static final List<AprilTag> kTags = List.of( 
//       // new AprilTag(1, new Pose3d(5, 2, 1, new Rotation3d(0, 0, 180)))
//       // 1 (0,114,67) pos x
//       // 2 (106,0,69.5) pos y 
//       // 3 (251,175.25,65) neg x
//       new AprilTag(1, new Pose3d(2.8956, 0     , 1.7018, new Rotation3d(0, 0, Math.PI/2))),
//       new AprilTag(2, new Pose3d(0     , 3.8202, 1.5494, new Rotation3d(0, 0, 0))),
//       new AprilTag(3, new Pose3d(4.4260, 6.3754, 1.6129, new Rotation3d(0, 0, 3 * Math.PI / 2)))
//     );
//     }
    

//     public static final AprilTagFieldLayout kAprilTagLayout = new AprilTagFieldLayout(AprilTags.kTags, 10, 10);

//     public static final class StdDevs {
//       public static final Matrix<N3, N1> kSingleTag = VecBuilder.fill(4, 4, 8);
//       public static final Matrix<N3, N1> kMultipleTags = VecBuilder.fill(0.5, 0.5, 1);
//     }
//   }

//   public static final class OIConstants { // eerrrm acksually itz IO
//     public static final class DriverController {
//       public static final int kPort = 0;
//       public static final double kDriveDeadband = 0.05;
//     }

//     public static final class OperatorController {
//       public static final int kPort = 1;
//     }
//   }

//   public static final double kFlyWheelRadius = 0.05;
//   public static final double kFeederWheelRadius = 0.025;
//   public static final double kIntakeWheelRadius = 0.019;
  
//   public static final double kShooterPositionFactor = 2 * Math.PI * kFlyWheelRadius;
//   public static final double kShooterVelocityFactor = kShooterPositionFactor / 60;

//   public static final double kFeederPositionFactor = 2 * Math.PI * kFeederWheelRadius;
//   public static final double kFeederVelocityFactor = kFeederPositionFactor / 60;

//   public static final double kIntakePositionFactor = 2 * Math.PI * kIntakeWheelRadius;
//   public static final double kIntakeVelocityFactor = kIntakePositionFactor / 60;

//   public static final class CANIDs {
//     public static final int coralArm;
//   }

//   public static final class PID {
//     public static final class CoralArm {
//       public static final double kP = 0;
//       public static final double kI = 0;
//       public static final double kD = 0;
//     }
//   }

//   public static final class MotorSpeeds {
    
//   }
// }
