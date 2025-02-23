package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import frc.robot.constants.DriveAutoConstants;
import frc.robot.constants.VisionConstants;

public class ReefSide {
    public final int kTagID;
    public final Rotation2d kHeading;

    public final Pose2d kLeftEndPose;
    public final Pose2d kRightEndPose;

    // public final Pose2d kLeftPathStartPose;
    // public final Pose2d kRightPathStartPose;

    public static enum ReefBranch {
        kLeft, kRight
    }

    public ReefSide(int tagID) {
        kTagID = tagID;
        
        Pose2d tagPose = VisionConstants.kAprilTagLayout.getTagPose(tagID).get().toPose2d();
        kHeading = tagPose.getRotation();

        Transform2d perpendicular = new Transform2d(kHeading.getCos(), kHeading.getSin(), Rotation2d.kZero);
        Transform2d parallel = new Transform2d(-kHeading.getSin(), kHeading.getCos(), Rotation2d.kZero);
        
        // lets see if this has any chance of working (yay)
        Pose2d centerPose = tagPose.plus(
            perpendicular.times(-DriveAutoConstants.kRobotDistanceFromCenter.in(Units.Meters))
        );

        // might have to switch these 2
        kLeftEndPose = centerPose.plus(
            parallel.times(DriveAutoConstants.kReefBranchDistanceFromCenter.in(Units.Meters))
        );
        kRightEndPose = centerPose.plus(
            parallel.times(-DriveAutoConstants.kReefBranchDistanceFromCenter.in(Units.Meters))
        );

        // kLeftPathStartPose = kLeftEndPose.plus(
        //     perpendicular.times(DriveAutoConstants.kReefBranchDistanceToPathingStart.in(Units.Meters))
        // );

        // kRightPathStartPose = kRightEndPose.plus(
        //     perpendicular.times(DriveAutoConstants.kReefBranchDistanceToPathingStart.in(Units.Meters))
        // );
    }

    public Pose2d getEndPose(ReefBranch reefBranch){
        return (reefBranch == ReefBranch.kLeft) ? kLeftEndPose : kRightEndPose;
    }
}
