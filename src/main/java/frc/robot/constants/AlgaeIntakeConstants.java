package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.controller.PIDConstants;

public class AlgaeIntakeConstants {
    public final class AlgaePivotConstants {
        public static final class AlgaePivotCANIDs {
            public static final int kPivot = 0;
        }

        public static final class PivotPID {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0.05;

            public static final PIDConstants pidConstants = new PIDConstants(
                kP,
                kI,
                kD,
                kFF
            );
        }

        public static final Rotation2d kPivotMinPosition = Rotation2d.fromDegrees(-180);
        public static final Rotation2d kPivotMaxPosition = Rotation2d.fromDegrees(180);
    }
}
