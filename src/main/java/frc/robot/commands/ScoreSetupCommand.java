package frc.robot.commands;

import java.util.Comparator;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer.subsystems;
import frc.robot.commands.drivecommands.AutoAlignDriveCommand;
import frc.robot.constants.CoralStates;
import frc.robot.constants.DriveAutoConstants;
import frc.robot.constants.ReefSides;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.drive.ReefSide;
import frc.robot.subsystems.drive.ReefSide.ReefBranch;

public class ScoreSetupCommand extends Command {

  private Supplier<CoralState> m_coralStateSup;
  private Supplier<ReefBranch> m_reefBranchSup;
  private CoralState m_coralState;
  private ReefBranch m_reefBranch;
  private ReefSide m_reefSide;
  private Distance m_pieceDisp;

  public ScoreSetupCommand(Supplier<CoralState> coralState, Supplier<ReefBranch> reefBranch) {
    m_coralStateSup = coralState;
    m_reefBranchSup = reefBranch;
    addRequirements(subsystems.drive, subsystems.elevator, subsystems.coralArm, subsystems.coralIntake);
  }

  public static ReefSide getClosestReefSideToPose(Pose2d startPose, ReefBranch reefBranch) {
    Optional<ReefSide> reefSideOp = ReefSides.kReefSides
      .stream()
      .sorted(Comparator.comparingDouble(
        side -> {
          Transform2d dist = side.getEndPose(reefBranch).minus(startPose);
          return dist.getX()*dist.getX() + dist.getY()*dist.getY();
        }
      ))
      .findFirst();

    return reefSideOp.orElse(null);   
  }

  // private ReefSide getSide() {
  //   Optional<AprilTag> closestTag = subsystems.vision.getClosestTag(DriveAutoConstants.kAutoDriveTagsIDs);

  //   if (closestTag.isEmpty()) {
  //     return getClosestReefSideToPose(subsystems.drive.getPose(), m_reefBranch);
  //   }

  //   int closestTagID = closestTag.get().ID;

  //   Optional<ReefSide> reefSideOp = ReefSides.kReefSides
  //     .stream()
  //     .filter(side -> side.kTagID == closestTagID)
  //     .findFirst();
    
  //   return reefSideOp.orElseGet(() -> getClosestReefSideToPose(subsystems.drive.getPose(), m_reefBranch));
  // }

  @Override
  public void initialize() {
    m_coralState = m_coralStateSup.get();
    m_reefBranch = m_reefBranchSup.get();

    m_reefSide = getClosestReefSideToPose(subsystems.drive.getPose(), m_reefBranch);
    System.out.println("Auto driving to tag : " + m_reefSide.kTagID);

    if (m_reefSide == null) return;
    if (!subsystems.coralIntake.hasPiece()) return;

    m_pieceDisp = subsystems.coralIntake.getPieceDispFromCenter();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    Pose2d targetPose = subsystems.drive.getReefTargetPose(m_coralState, m_reefSide, m_reefBranch, m_pieceDisp);

    subsystems.drive.pathfindToPoseCommand(targetPose)
      .andThen(new InstantCommand(subsystems.drive::setSingleTagPoseEstimation, subsystems.drive))
      .andThen(new AutoAlignDriveCommand(targetPose).withTimeout(2))
      .andThen(new AutoAlignDriveCommand(targetPose)
        .alongWith(
          new SetCoralStateCommand(m_coralState).withTimeout(2)
        )
      )
      .schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
