package frc.robot.constants;

import java.util.Map;
import static java.util.Map.entry;

import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevels;

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

        public static final Map<ElevatorLevels, Double> levelHeights = Map.ofEntries(
            entry(ElevatorLevels.L1, null),
            entry(ElevatorLevels.L2, null),
            entry(ElevatorLevels.L3, null),
            entry(ElevatorLevels.L4, null)
        );
}
