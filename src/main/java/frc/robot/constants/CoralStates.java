package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.coral.CoralState;

public class CoralStates {
    public static final CoralState kRest = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-60)
    );

    public static final CoralState kStation = new CoralState(
        Units.Inches.of(12.5),
        Rotation2d.fromDegrees(-45)
    );

    public static CoralState kL1 = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-45)
    );

    public static final CoralState kL2 = new CoralState(
        Units.Inches.of(8),
        Rotation2d.fromDegrees(-63)
    );

    public static final CoralState kL3 = new CoralState(
        Units.Inches.of(4),
        Rotation2d.fromDegrees(130)
    );

    public static final CoralState kL4 = new CoralState(
        Units.Inches.of(28),
        Rotation2d.fromDegrees(135)
    );
}
