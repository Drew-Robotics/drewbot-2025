package frc.robot.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.constants.ReefSides;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.drive.ReefSide;
import frc.robot.subsystems.drive.ReefSide.ReefBranch;

public class ScoreCommands {
    private static CoralState m_coralState = CoralStates.kL1;
    private static ReefSide m_reefSide;
    private static ReefBranch m_reefBranch = ReefBranch.Left;

    public void setReefBranch(ReefBranch reefBranch) {
        m_reefBranch = reefBranch;
    }

    public void setCoralState(CoralState coralState) {
        m_coralState = coralState;
    }


    public static Command scoreCommand() {
        // m_reefSide = subsystems.vision.getClosestTag(DriveAutoConstants.kAutoDriveTagsIDs);
        m_reefSide = ReefSides.kFront;

        if (!subsystems.coralIntake.hasPiece())
            return Commands.none();

        Distance pieceDisp = subsystems.coralIntake.getPieceDispFromCenter();

        return subsystems.drive.pathfindToCoralCommand(m_reefSide, m_reefBranch, pieceDisp)
            .andThen(new SetCoralStateCommand(m_coralState))
            .andThen(new CoralOuttakeCommand())
            .andThen(new SetCoralStateCommand(CoralStates.kRest));
    }
}
