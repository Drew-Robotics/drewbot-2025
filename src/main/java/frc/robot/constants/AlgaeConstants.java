package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class AlgaeConstants {
    public static final class CANIDs {
        public static final int kArm = 30;
        public static final int kIntake = 31;
        public static final int kSensor = 32;
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

    public static final class ConversionFactors {
        public static final class Arm {
            public static Angle kPositionConversionFactor = Units.Radians.of(2 * Math.PI);
            public static AngularVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Minutes);
        };

        public static final class Intake {
            public static Distance kWheelRadius = Units.Inches.of(0);
            public static Distance kPositionConversionFactor = kWheelRadius.times(2 * Math.PI); 
            public static LinearVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Minutes);
        };
    }

    public static Current kArmCurrentLimit = Units.Amps.of(0);
    public static Current kIntakeCurrentLimit = Units.Amps.of(0);
    
    public static final Rotation2d kPivotMinPosition = Rotation2d.fromDegrees(-180);
    public static final Rotation2d kPivotMaxPosition = Rotation2d.fromDegrees(180);

    public static final Distance kSensorTripDistance = Centimeters.of(0);
    
    public static final Rotation2d kArmRestingAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d kArmExtendedAngle = Rotation2d.fromDegrees(0);
}
