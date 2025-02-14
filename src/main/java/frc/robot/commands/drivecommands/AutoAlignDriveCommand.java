package frc.robot.commands.drivecommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotContainer.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.Comparator;

public class AutoAlignDriveCommand extends DriveCommand {
  List<Integer> m_acceptedTagsIDs = List.of(0, 1, 2, 3);
    
  public AutoAlignDriveCommand(DoubleSupplier xVel, DoubleSupplier yVel) {
    super(xVel, yVel, null);
    addRequirements(subsystems.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Optional<Rotation2d> tagRot = getClosestTagRotation();
    if (!tagRot.isEmpty()) {
      ChassisSpeeds speeds = 
        subsystems.drive.getChassisSpeedOnRotationControl(
          getXVelocity(), getXVelocity(), tagRot.get()
        );

      subsystems.drive.setChassisSpeeds(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public Optional<Rotation2d> getClosestTagRotation() {
    List<AprilTag> tags = subsystems.vision.getSeenTags().stream()
      .filter(tag -> m_acceptedTagsIDs.contains(tag.ID)).toList();

    if (tags.size() == 0){
      // this is why it's in its own method; returning from execute is bad practice. <- Reply: Your bad practice (GOTTEM ! ! ! !)
      return Optional.empty();
    }

    Translation3d robotTranslation3d = subsystems.drive.getPose3d().getTranslation();
    tags.sort(Comparator.comparingDouble(
      tag -> tag.pose.getTranslation().getDistance(robotTranslation3d)
    ));

    Angle closestRotation = tags.get(0).pose.getRotation().getMeasureZ();
    return Optional.of(Rotation2d.fromRadians(closestRotation.in(Units.Radians)));
  }
}