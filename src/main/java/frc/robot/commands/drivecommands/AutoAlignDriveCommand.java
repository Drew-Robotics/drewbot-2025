package frc.robot.commands.drivecommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.DriveAutoConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AutoAlignDriveCommand extends TurnToAngleCommand {
  
  public AutoAlignDriveCommand(DoubleSupplier xVel, DoubleSupplier yVel) {
    super(xVel, yVel, new Supplier<Rotation2d>() {
      @Override
      public Rotation2d get() {
        return subsystems.vision.getClosestTag(DriveAutoConstants.kAutoAlignTagsIDs)
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