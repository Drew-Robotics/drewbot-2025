package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class CoralConstants {

    public static final class CANIDs {
        public static final int kElevatorLeft = 0;
        public static final int kElevatorRight = 0;
        public static final int kCoralArm = 0;
        public static final int kCoralIntake = 0;
    }

    public static final class PID{
        public static final class Elevator {
            public static final double kP = 0.05;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0.05;

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
    
    public static Current kElevatorCurrentLimit = Units.Amps.of(0);
    public static Current kArmCurrentLimit = Units.Amps.of(0);
    public static Current kIntakeCurrentLimit = Units.Amps.of(0);


    public static final class ReefHeights {
        public static final Distance L1 = Units.Inches.of(0);
        public static final Distance L2 = Units.Inches.of(0);
        public static final Distance L3 = Units.Inches.of(0);
        public static final Distance L4 = Units.Inches.of(0);
    }

    public static final class ArmHeightConversion {
        public static final Distance kElevatorMinHeight = Units.Meters.of(0);
        public static final Distance kElevatorMaxHeight = Units.Meters.of(0);
        
        public static final Angle kMinRotations = Units.Rotations.of(0);
        public static final Angle kMaxRotations = Units.Rotations.of(0);
    }
}
