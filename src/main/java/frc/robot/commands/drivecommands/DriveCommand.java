package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private DoubleSupplier m_xVelocity;
  private DoubleSupplier m_yVelocity;
  private DoubleSupplier m_rot;

  public DriveCommand(
    DoubleSupplier xVel, 
    DoubleSupplier yVel, 
    DoubleSupplier rot
  ) {
    m_xVelocity = xVel;
    m_yVelocity = yVel;
    m_rot = rot;

    addRequirements(subsystems.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    super.execute();

    subsystems.drive.drive(
      m_xVelocity.getAsDouble(), 
      m_yVelocity.getAsDouble(), 
      m_rot.getAsDouble()
    );
  }

  @Override
  public void end(boolean interrupted) {
    subsystems.drive.drive(0,0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double getXVelocity() {
    

    return m_xVelocity.getAsDouble();
  }

  public double getYVelocity() {
    return m_yVelocity.getAsDouble();
  }
}