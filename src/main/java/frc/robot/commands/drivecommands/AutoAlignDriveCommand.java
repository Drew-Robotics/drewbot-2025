package frc.robot.commands.drivecommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.RobotContainer.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AutoAlignDriveCommand extends TurnToAngleCommand {
  private static final List<Integer> kAcceptedTagsIDs = List.of(0, 1, 2, 3);
  
  public AutoAlignDriveCommand(DoubleSupplier xVel, DoubleSupplier yVel) {
    super(xVel, yVel, new Supplier<Rotation2d>() {
      @Override
      public Rotation2d get() {
        return subsystems.vision.getClosestTag(kAcceptedTagsIDs)
          .map(tag -> Rotation2d.fromRadians(tag.pose.getRotation().getMeasureZ().in(Units.Radians)))
          .orElse(subsystems.drive.getGyroYaw());
      }
    });
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}