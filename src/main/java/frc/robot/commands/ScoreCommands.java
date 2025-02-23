package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.drive.ReefSide;
import frc.robot.subsystems.drive.ReefSide.ReefBranch;

public class ScoreCommands {
    public static Command scoreCommand(CoralState coralState, ReefSide reefSide, ReefBranch reefBranch) {
        if (!subsystems.coralIntake.hasPiece())
            return Commands.none();

        Distance pieceDisp = subsystems.coralIntake.getPieceDispFromCenter();

        return subsystems.drive.pathfindToCoralCommand(reefSide, reefBranch, pieceDisp)
            .andThen(new SetCoralStateCommand(coralState))
            .andThen(new CoralOuttakeCommand())
            .andThen(new SetCoralStateCommand(CoralStates.kRest));
    }
}
