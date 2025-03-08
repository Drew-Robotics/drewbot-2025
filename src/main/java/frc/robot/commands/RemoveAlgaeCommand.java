package frc.robot.commands;

import java.util.Comparator;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
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

public class RemoveAlgaeCommand extends Command {

  private Supplier<CoralState> m_coralStateSup;
  private CoralState m_coralState;
  private ReefSide m_reefSide;

  public RemoveAlgaeCommand(Supplier<CoralState> coralState) {
    m_coralStateSup = coralState;
    addRequirements(subsystems.drive, subsystems.elevator, subsystems.coralArm, subsystems.coralIntake);
  }

  public static ReefSide getClosestReefSideToPose(Pose2d startPose) {
    Optional<ReefSide> reefSideOp = ReefSides.kReefSides
      .stream()
      .sorted(Comparator.comparingDouble(
        side -> {
          Transform2d dist = side.kCenterPose.minus(startPose);
          return dist.getX()*dist.getX() + dist.getY()*dist.getY();
        }
      ))
      .findFirst();

    return reefSideOp.orElse(null);   
  }


  @Override
  public void initialize() {
    m_coralState = m_coralStateSup.get();

    m_reefSide = getClosestReefSideToPose(subsystems.drive.getPose());
    System.out.println("Removing algae at tag : " + m_reefSide.kTagID);

    if (m_reefSide == null) return;
    if (subsystems.coralIntake.hasPiece()) return;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    Pose2d targetPose = subsystems.drive.getReefTargetPose(m_coralState, m_reefSide, ReefBranch.Center, Units.Meters.zero());

    subsystems.drive.pathfindToPoseCommand(targetPose)
      .andThen(new InstantCommand(subsystems.drive::setSingleTagPoseEstimation, subsystems.drive))
      .andThen(new AutoAlignDriveCommand(targetPose).withTimeout(2))
      .andThen(new AutoAlignDriveCommand(targetPose)
        .withDeadline(
          new SetCoralStateCommand(m_coralState).withTimeout(2)
        )
      .andThen(new CoralOuttakeCommand().withTimeout(1))
      .andThen(new SetCoralStateCommand(CoralStates.kRest).withTimeout(1))
      )
      .schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
