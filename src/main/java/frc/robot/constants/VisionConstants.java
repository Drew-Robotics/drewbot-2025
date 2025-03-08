package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
  
    public static final class CameraNames {
      public static final String kFrontLeft = "frontLeft";
      public static final String kFrontRight = "frontRight";
      public static final String kBackLeft = "backLeft";
      public static final String kBackRight = "backRight";

      public static final String kLLFront = "llFront";
      public static final String kLLBack = "llBack";
    }
    public static final class CameraTransforms {
      // Measure from the center line to the camera, not the 3D center of the robot.

      // For Arducams
      public static final Distance lengthWidth = Inches.of(14);
      public static final Distance height = Inches.of(9.16);
      public static final Angle pitch = Degrees.of(-20);
      public static final Angle yawOffset = Degrees.of(20);

      public static final Transform3d kFrontLeft = new Transform3d(
        Units.Inches.of(-11.48).in(Meters), // fb
        Units.Inches.of(-11).in(Meters), // lr
        height.in(Meters), 
        new Rotation3d(0, pitch.in(Radians), - Math.PI + yawOffset.in(Radians))
      );
      public static final Transform3d kFrontRight = new Transform3d(
        Units.Inches.of(-11.48).in(Meters), // fb
        Units.Inches.of(11).in(Meters), // lr
        height.in(Meters), 
        new Rotation3d(0, pitch.in(Radians), Math.PI - yawOffset.in(Radians))
      );
      public static final Transform3d kBackLeft = new Transform3d(
        Units.Inches.of(11.48).in(Meters), // fb
        Units.Inches.of(-11).in(Meters), // lr
        height.in(Meters), 
        new Rotation3d(0, pitch.in(Radians), - yawOffset.in(Radians))
      );
      public static final Transform3d kBackRight = new Transform3d(
        Units.Inches.of(11.48).in(Meters), // fb
        Units.Inches.of(11).in(Meters), // lr
        height.in(Meters), 
        new Rotation3d(0, pitch.in(Radians), yawOffset.in(Radians))
      );

      // Limelights

      public static final Transform3d kLLFront = new Transform3d(
        Units.Inches.of(-5.59).in(Meters), // fb
        Units.Inches.of(-8.54).in(Meters), // lr
        Units.Inches.of(13.02).in(Meters),
        new Rotation3d(0, Units.Degree.of(5).in(Units.Radians), Units.Degree.of(180 - 25).in(Units.Radians))
      );

      public static final Transform3d kLLBack = new Transform3d(
        Units.Inches.of(4.42).in(Meters), // fb
        Units.Inches.of(0).in(Meters), // lr
        Units.Inches.of(36.75).in(Meters),
        new Rotation3d(0, Units.Degree.of(45).in(Units.Radians), 0)
      );

      // public static final Transform3d kFrontLeft = new Transform3d(
      //   -lengthWidth.in(Meters),
      //   -lengthWidth.in(Meters),
      //   height.in(Meters), 
      //   new Rotation3d(0, pitch.in(Radians), - Math.PI + yawOffset.in(Radians))
      // );
      // public static final Transform3d kFrontRight = new Transform3d(
      //   -lengthWidth.in(Meters),
      //   lengthWidth.in(Meters),
      //   height.in(Meters), 
      //   new Rotation3d(0, pitch.in(Radians), Math.PI - yawOffset.in(Radians))
      // );
      // public static final Transform3d kBackLeft = new Transform3d(
      //   lengthWidth.in(Meters),
      //   -lengthWidth.in(Meters),
      //   height.in(Meters), 
      //   new Rotation3d(0, pitch.in(Radians), - yawOffset.in(Radians))
      // );
      // public static final Transform3d kBackRight = new Transform3d(
      //   lengthWidth.in(Meters),
      //   lengthWidth.in(Meters),
      //   height.in(Meters), 
      //   new Rotation3d(0, pitch.in(Radians), yawOffset.in(Radians))
      // );

      // public static final Transform3d kBackRight = new Transform3d(
      //   lengthWidth.in(Meters),
      //   lengthWidth.in(Meters),
      //   height.in(Meters), 
      //   new Rotation3d(0, pitch.in(Radians), yawOffset.in(Radians))
      // );
    }

    // public static final class AprilTags {
    // public static final List<AprilTag> kTags = List.of( 
    //   // new AprilTag(1, new Pose3d(5, 2, 1, new Rotation3d(0, 0, 180)))
    //   // 1 (0,114,67) pos x
    //   // 2 (106,0,69.5) pos y 
    //   // 3 (251,175.25,65) neg x
    //   new AprilTag(1, new Pose3d(2.8956, 0     , 1.7018, new Rotation3d(0, 0, Math.PI/2))),
    //   new AprilTag(2, new Pose3d(0     , 3.8202, 1.5494, new Rotation3d(0, 0, 0))),
    //   new AprilTag(3, new Pose3d(4.4260, 6.3754, 1.6129, new Rotation3d(0, 0, 3 * Math.PI / 2)))
    // );
    // }

    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final class StdDevs {
      public static final Matrix<N3, N1> kSingleTag = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultipleTags = VecBuilder.fill(0.5, 0.5, 1);
    }
  }
