package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

    public static final class ElevatorCANIDs {
        public static final int kElevator = 0;
    }

    public static final class ElevatorPID {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        
    }

    public static final class Measurements {
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;
        public static final double L4 = 0;
    }

    public static final Distance kElevatorMinHeight = Units.Meters.of(0);
    public static final Distance kElevatorMaxHeight = Units.Meters.of(0);

    public static final Angle kMinRotations = Units.Rotations.of(0);
    public static final Angle kMaxRotations = Units.Rotations.of(0);
}
