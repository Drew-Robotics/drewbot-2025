package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;

public class AlgaeHoldCommand extends Command {
  public AlgaeHoldCommand() {
    addRequirements(subsystems.algaeArm, subsystems.algaeIntake);
  }

  @Override
  public void initialize() {
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmHoldAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kHoldVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmRestingAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kRestVoltage);
  }

  @Override
  public boolean isFinished() {
    return !subsystems.algaeIntake.hasPiece();
  }
}
