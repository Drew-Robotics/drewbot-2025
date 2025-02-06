package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTag;

import frc.robot.RobotContainer.subsystems;

import java.util.List;
import java.util.Comparator;

public class AutoAlignCommand extends Command{
  List<Integer> m_acceptedTagsIDs = List.of(0,1,2,3);
    
public AutoAlignCommand(){
  addRequirements(subsystems.drive);
}

@Override
public void initialize() {}

@Override
public void execute() {
  List<AprilTag> tags = subsystems.vision.getSeenTags().stream()
    .filter(tag -> m_acceptedTagsIDs.contains(tag.ID))
    .sorted(Comparator.comparingDouble(
      tag -> tag.pose.
    ));
}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}