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
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmExtendedAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kIntakeVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmRestingAngle);
    subsystems.algaeIntake.setVoltage(Units.Volts.zero());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
