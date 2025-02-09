package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.coral.CoralState;

public class CoralStates {
    public static CoralState kRest = new CoralState(
        Units.Meters.of(0),
        Rotation2d.fromDegrees(0)
    );
}
