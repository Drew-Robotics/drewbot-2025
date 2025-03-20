package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;

public class AlgaeIntakeCommand extends Command {
  public AlgaeIntakeCommand() {
    addRequirements(subsystems.algaeArm, subsystems.algaeIntake);
  }

  @Override
  public void initialize() {
    if(subsystems.algaeIntake.hasPiece()) {
      end(true);
      return;
    }
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmIntakeAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kIntakeVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      subsystems.algaeArm.toRestState();
      subsystems.algaeIntake.setVoltage(AlgaeConstants.kRestVoltage);
      return;
    }

    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmHoldAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kHoldVoltage);
    new AlgaeHoldCommand().schedule();
  }

  @Override
  public boolean isFinished() {
    return subsystems.algaeIntake.hasPiece();
  }
}
