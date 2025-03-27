package frc.robot.constants;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.coral.CoralState;

public class CoralStates {
    public static final CoralState kRest = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-60),
        false,
        "Rest"
    );

    public static final CoralState kStation = new CoralState(
        Units.Inches.of(12.5),
        Rotation2d.fromDegrees(-45),
        false,
        "Feed Station"
    );

    public static final CoralState kHasCoralRest = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(130),
        false,
        "Has Coral Rest"
    );


    public static final CoralState kAlgaeL2 = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-17.5),
        false,
        "Algae L2"
    );

    public static final CoralState kAlgaeL3 = new CoralState(
        Units.Inches.of(8),
        Rotation2d.fromDegrees(0),
        false,
        "Algae L3"
    );

    public static CoralState kL1 = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-63),
        false,
        "Reef L1"
    );

    public static final CoralState kL2 = new CoralState(
        Units.Inches.of(6),
        Rotation2d.fromDegrees(-63),
        false,
        "Reef L2"
    );

    public static final CoralState kL3 = new CoralState(
        Units.Inches.of(2),
        Rotation2d.fromDegrees(130),
        true,
        "Reef L3"
    );

    public static final CoralState kL4 = new CoralState(
        Units.Inches.of(27),
        Rotation2d.fromDegrees(135),
        true,
        "Reef L4"
    );

    public static final CoralState kClimberUp = new CoralState(
        Units.Inches.of(18.5),
        Rotation2d.fromDegrees(130),
        true,
        "Climber Up"
    );

    public static final CoralState kClimberHold = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(120),
        true,
        "Climber Hold"
    );

    public static final CoralState kClimberHoldArmUp = new CoralState(
        Units.Inches.of(0),
        Rotation2d.fromDegrees(-60),
        true,
        "Climber Hold Arm Up"
    );


    public static final List<CoralState> kLoggedStates = List.of(
        // kRest, kStation, 
        kAlgaeL2, kAlgaeL3,
        kL1, kL2, kL3, kL4
        // kClimberUp, kClimberHold
    );
}
