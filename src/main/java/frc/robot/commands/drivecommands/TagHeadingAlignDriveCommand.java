package frc.robot.commands.drivecommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.ReefSides;
import frc.robot.subsystems.drive.ReefSide;

import java.util.Comparator;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class TagHeadingAlignDriveCommand extends TurnToAngleCommand {
  
  public TagHeadingAlignDriveCommand(DoubleSupplier xVel, DoubleSupplier yVel) {
    super(xVel, yVel, new Supplier<Rotation2d>() {
      @Override
      public Rotation2d get() {

        Optional<ReefSide> reefSideOp = ReefSides.kReefSides
          .stream()
          .sorted(Comparator.comparingDouble(
            side -> {
              Transform2d dist = side.kCenterPose.minus(subsystems.drive.getPose());
              return dist.getX()*dist.getX() + dist.getY()*dist.getY();
            }
          ))
          .findFirst();
        
        if (reefSideOp.isEmpty())
          return subsystems.drive.getGyroYaw();
        
        return reefSideOp.get().kHeading;
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