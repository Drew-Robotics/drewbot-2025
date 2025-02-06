package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
  public static final class ArmPID {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.05;
  }

  public static final class ArmCANIDs {
    public static final int kArm = 0;
  }

  public static final Rotation2d kArmMinPosition = Rotation2d.fromDegrees(-180);
  public static final Rotation2d kArmMaxPosition = Rotation2d.fromDegrees(180);
}
