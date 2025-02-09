package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;

public class AlgaeConstants {
    public static final class CANIDs {
        public static final int kArm = 0;
        public static final int kIntake = 0;
    }

    public static final class PID {
        public static final class Arm {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0.05;

        }
    
        public static final class Intake {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0.05;
        }
    }

    public static Current kArmCurrentLimit = Units.Amps.of(0);
    public static Current kIntakeCurrentLimit = Units.Amps.of(0);
    
    public static final Rotation2d kPivotMinPosition = Rotation2d.fromDegrees(-180);
    public static final Rotation2d kPivotMaxPosition = Rotation2d.fromDegrees(180);
}
