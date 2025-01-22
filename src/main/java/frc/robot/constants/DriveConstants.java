import edu.wpi.first.units.Units;

public static final class DriveConstants {

  public static final Measure<Distance> kWheelBase = Units.Inches.of(24.5);
  public static final Measure<Distance> kTrackWidth = Units.Inches.of(24.5);

  public static final Boolean kGyroReversed = true;

  public static final Boolean kFieldOriented = true;

  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2)),
    new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2).negate()),
    new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2)),
    new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2).negate())
  );

  public static final class CanIDs {
    public static final int kFrontLeftDriving = 1;
    public static final int kBackLeftDriving = 5;
    public static final int kFrontRightDriving = 3;
    public static final int kBackRightDriving = 7;

    public static final int kFrontLeftTurning = 2;
    public static final int kBackLeftTurning = 6;
    public static final int kFrontRightTurning = 4;
    public static final int kBackRightTurning = 8;
  }

  public static final class AngularOffsets {
    public static final Rotation2d kFrontLeft = new Rotation2d(-Math.PI / 2);
    public static final Rotation2d kFrontRight = new Rotation2d(0);
    public static final Rotation2d kBackLeft = new Rotation2d(Math.PI);
    public static final Rotation2d kBackRight = new Rotation2d(Math.PI / 2);
  }

  public static final class MaxVels {
    public static final Measure<Velocity<Distance>> kTranslationalVelocity = Units.MetersPerSecond.of(4.5);
    public static final Measure<Velocity<Angle>> kRotationalVelocity = Units.RadiansPerSecond.of(2 * Math.PI);
  }

  public static final class SlewRate {
    public static final double kMag = 2;
    public static final double kDir = 2;
    public static final double kRot = 2;
  }
}