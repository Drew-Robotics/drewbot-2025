package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.coral.CoralState;

public class CoralStates {
    public static CoralState kRest = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(0)
    );

    public static CoralState kL1 = new CoralState(
        Units.Inches.of(7),
        Rotation2d.fromDegrees(0)
    );

    public static CoralState kL2 = new CoralState(
        Units.Inches.of(14),
        Rotation2d.fromDegrees(0)
    );

    public static CoralState kL3 = new CoralState(
        Units.Inches.of(21),
        Rotation2d.fromDegrees(0)
    );

    public static CoralState kL4 = new CoralState(
        Units.Inches.of(28),
        Rotation2d.fromDegrees(0)
    );
}
