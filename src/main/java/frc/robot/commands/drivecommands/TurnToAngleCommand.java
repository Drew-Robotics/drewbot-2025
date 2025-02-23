package frc.robot.commands.drivecommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurnToAngleCommand extends DriveCommand {
  List<Integer> m_acceptedTagsIDs = List.of(0, 1, 2, 3);

  Supplier<Rotation2d> m_setAngle;
    
  public TurnToAngleCommand(DoubleSupplier xVel, DoubleSupplier yVel, Rotation2d angle) {
    this(xVel, yVel, () -> angle);
  }

  public TurnToAngleCommand(DoubleSupplier xVel, DoubleSupplier yVel, Supplier<Rotation2d> angle) {
    super(xVel, yVel, null);
    m_setAngle = angle;
    addRequirements(subsystems.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = 
      subsystems.drive.getChassisSpeedOnRotationControl(
        getXVelocity(), getXVelocity(), m_setAngle.get()
      );

    subsystems.drive.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}