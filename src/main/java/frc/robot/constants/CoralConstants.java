package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity; 
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralConstants {
    public static Current kElevatorCurrentLimit = Units.Amps.of(60);
    public static Current kArmCurrentLimit = Units.Amps.of(5);
    public static Current kIntakeCurrentLimit = Units.Amps.of(5);

    public static boolean kElevatorLeftMotorInverted = false;
    public static boolean kElevatorRightMotorInverted = true;

    public static Rotation2d kMinArmAngle = Rotation2d.fromDegrees(-180);
    public static Rotation2d kMaxArmAngle = Rotation2d.fromDegrees(180);

    public static boolean kCoralArmMotorInverted = false;
    public static boolean kCoralIntakeMotorInverted = false;

    public static LinearVelocity kIntakeSurfaceVelocity = Units.MetersPerSecond.of(1);
    public static LinearVelocity kOuttakeSurfaceVelocity = Units.MetersPerSecond.of(-1);
    
    public static final class IdleModes {
        public static final IdleMode kArm = IdleMode.kCoast;
        public static final IdleMode kIntake = IdleMode.kCoast;
        public static final IdleMode kElevator = IdleMode.kBrake;
    }

    public static final class CANIDs {
        public static final int kElevatorLeft = 10;
        public static final int kElevatorRight = 11;
        public static final int kElevatorEncoder = 22;
        public static final int kCoralArm = 20;
        public static final int kCoralIntake = 21;
    }

    public static final class PID {
        public static final class Elevator {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.005;
            public static final double kOutput = 0.6;
        }
    
        public static final class CoralArm {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0.05;
        }
    
        public static final class CoralIntake {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class ConversionFactors {
        public static final class Arm {
            public static Angle kPositionConversionFactor = Units.Radians.of(2 * Math.PI);
            public static AngularVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Minutes);
        };
        /*
        Say, can you see
        By the dawn's early light
        What so proudly we hailed
        At the twilight's last gleaming?
        Whose broad stripes and bright stars
        Through the perilous fight
        O'er the ramparts we watched
        Were so gallantly, yeah, streaming?
        And the rockets' red glare
        The bombs bursting in air
        Gave proof through the night
        That our flag was still there
        O say, does that star-spangled banner yet wave
        O'er the land of the free and the home of the brave
        * CAAAAAW WHAT THE FRICK IS A KILOMETER
        * \\             //
        *  \\\' ,      / //
        *   \\\//,   _/ //,
        *    \_-//' /  //,
        *      \ ///  //`
        *     /  ;  \\\`__/_
        *    /,)-^ _\` \\\
        *    (/   \\ //\\
        *        // _//\\\\
        *       ((` ((
        */
        public static final class Intake {
            public static Distance kWheelRadius = Units.Inches.of(1.5);
            public static Distance kPositionConversionFactor = kWheelRadius.times(2 * Math.PI); 
            public static LinearVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Minutes);
        };
    }

    public static final class ArmHeightConversion {
        public static final Distance kElevatorMinHeight = Units.Inches.of(0);
        public static final Distance kElevatorMaxHeight = Units.Inches.of(28);
        
        public static final Angle kMinRotations = Units.Rotations.of(0);
        public static final Angle kMaxRotations = Units.Rotations.of(74.36);
    }
}
