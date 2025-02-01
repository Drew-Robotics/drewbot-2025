package frc.robot.constants;

import frc.robot.controller.PIDConstants;

public class ElevatorConstants {

    public static final class ElevatorCANIDs {
        public static final int kElevator = 0;
    }

    public static final class ElevatorPID {
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        
        public static final PIDConstants pidConstants = new PIDConstants(
            kP,
            kI,
            kD
        );
    }

    public static final class Measurements {
        public static final double L1 = 0;
        public static final double L2 = 0;
        public static final double L3 = 0;
        public static final double L4 = 0;
    }
}
