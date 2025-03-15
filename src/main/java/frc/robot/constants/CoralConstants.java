package frc.robot.constants;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralConstants {
    public static Current kElevatorCurrentLimit = Units.Amps.of(60);
    public static Current kArmCurrentLimit = Units.Amps.of(40);
    public static Current kIntakeCurrentLimit = Units.Amps.of(20);

    public static boolean kElevatorLeftMotorInverted = false;
    public static boolean kElevatorRightMotorInverted = true;
    
    public static AngularVelocity kElevatorRestVelocity = Units.Rotations.per(Units.Minutes).of(50);
    public static double kRestTimerSeconds = 0.5;

    public static Rotation2d kMinArmAngle = Rotation2d.fromDegrees(-180);
    public static Rotation2d kMaxArmAngle = Rotation2d.fromDegrees(180);

    public static boolean kCoralArmMotorInverted = false;
    public static boolean kCoralIntakeMotorInverted = false;

    public static Voltage kIntakeVoltage = Units.Volts.of(2);
    public static Voltage kOuttakeVoltage = Units.Volts.of(-5);
    public static Voltage kHoldVoltage = Units.Volts.of(1); // TODO : test this
    public static Voltage kAlgaeRemoveVoltage = Units.Volts.of(5);

    public static Distance kElevatorAtStatePositionTolerance = Centimeter.of(1);
    public static AngularVelocity kElevatorAtStateVelocityTolerance = Units.Rotations.per(Units.Minutes).of(50);

    public static Distance kIntakeWidth = Units.Inches.of(22);
    public static Distance kMaxTOFReading = Units.Inches.of(19);
    public static Distance kCoralWidth = Units.Inches.of(4.5);

    public static int kIntakeTOFRangeOfInterest = 2;
    public static Distance kTOFCorrection = Units.Millimeters.of(52);

    public static Distance kCenteredCoralReading = kIntakeWidth
        .minus(kCoralWidth)
        .times(0.5);

    public static Angle kCoralArmAtStatePositionTolerance = Degrees.of(10);
    public static AngularVelocity kCoralArmAtStateVelocityTolerance = DegreesPerSecond.of(10);
    
    public static final class IdleModes {
        public static final IdleMode kArm = IdleMode.kBrake;
        public static final IdleMode kIntake = IdleMode.kCoast;
        public static final IdleMode kElevator = IdleMode.kBrake;
    }

    public static final class CANIDs {
        public static final int kElevatorLeft = 10;
        public static final int kElevatorRight = 11;

        public static final int kCoralArm = 20;
        public static final int kCoralArmEncoder = 22;

        public static final int kCoralIntake = 21;
        public static final int kCoralIntakeTimeOfFlight = 23;
    }

    public static final class PID {
        public static final class Elevator {
            public static final double kP = 0.06;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.005;
            public static final double kOutput = 1;
        }
    
        public static final class CoralArm {
            public static final double kP = 12; // 1.5
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kG = 0.7;
            public static final AngularVelocity kMaxVel = Units.DegreesPerSecond.of(300);
            public static final AngularAcceleration kMaxAccel = Units.DegreesPerSecondPerSecond.of(1000);
        }
    
        public static final class CoralIntake {
            public static final double kP = 0.3;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class ConversionFactors {
        public static final class Arm {
            public static double kGearReduction = (64d/28d) * (64d/16d) * 3d * 5d;
            public static Angle kPositionConversionFactor = Units.Rotations.of(1);
            public static AngularVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Seconds);
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
            // public static Distance kWheelRadius = Units.Inches.of(1.5);
            // public static Distance kPositionConversionFactor = kWheelRadius.times(2 * Math.PI); 
            // public static LinearVelocity kVelocityConversionFactor = kPositionConversionFactor.per(Units.Minutes);
        };
    }

    public static final class ElevatorHeightConversion {
        public static final Distance kElevatorMinHeight = Units.Inches.of(0);
        public static final Distance kElevatorMaxHeight = Units.Inches.of(28);
        
        public static final Angle kMinRotations = Units.Rotations.of(0);
        public static final Angle kMaxRotations = Units.Rotations.of(74.56);
    }
}
