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

    public final Pose2d kTagPose2d;
    public final Pose2d kCenterPose;


    public final Transform2d kPerpNorm;
    public final Transform2d kParaNorm;

    public static enum ReefBranch {
        Left, Right, Center
    }

    public ReefSide(int tagID) {
        kTagID = tagID;
        
        if (VisionConstants.kAprilTagLayout.getTagPose(tagID).isEmpty()) {
            kHeading = Rotation2d.kZero;

            kLeftEndPose = Pose2d.kZero;
            kRightEndPose = Pose2d.kZero;
            kCenterPose = Pose2d.kZero;
            kTagPose2d = Pose2d.kZero;

            kPerpNorm = Transform2d.kZero;
            kParaNorm = Transform2d.kZero;
            return;
        }

        kTagPose2d = VisionConstants.kAprilTagLayout.getTagPose(tagID).get().toPose2d();
        kHeading = kTagPose2d.getRotation();

        kPerpNorm = new Transform2d(-1, 0, Rotation2d.kZero);
        kParaNorm = new Transform2d(0, -1, Rotation2d.kZero);
        
        // lets see if this has any chance of working (yay)
        kCenterPose = kTagPose2d.plus(
            kPerpNorm.times(-DriveAutoConstants.kRobotDistanceFromCenter.in(Units.Meters))
        );

        // might have to switch these 2
        kLeftEndPose = kCenterPose.plus(
            kParaNorm.times(DriveAutoConstants.kReefBranchDistanceFromCenter.in(Units.Meters))
        );
        kRightEndPose = kCenterPose.plus(
            kParaNorm.times(-DriveAutoConstants.kReefBranchDistanceFromCenter.in(Units.Meters))
        );

        // kLeftPathStartPose = kLeftEndPose.plus(
        //     perpendicular.times(DriveAutoConstants.kReefBranchDistanceToPathingStart.in(Units.Meters))
        // );

        // kRightPathStartPose = kRightEndPose.plus(
        //     perpendicular.times(DriveAutoConstants.kReefBranchDistanceToPathingStart.in(Units.Meters))
        // );
    }

    public Pose2d getEndPose(ReefBranch reefBranch){
        switch (reefBranch){
            case Left:
                return kLeftEndPose;
            case Right:
                return kRightEndPose;
            case Center: // technically not needed but whatever
                return kCenterPose;
            default:
                return kCenterPose;
        }
    }
}
