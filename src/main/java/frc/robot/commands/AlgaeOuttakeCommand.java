package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;

public class AlgaeOuttakeCommand extends Command {
  public AlgaeOuttakeCommand() {
    addRequirements(subsystems.algaeArm, subsystems.algaeIntake);
  }

  @Override
  public void initialize() {
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmEjectAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kEjectVoltage);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    // could be interrupted (we cancel by not pressing), or not (the piece isn't detected anymore)
    // both have the same behavior, i just wanted to clairify in comments since this isnt super recommended
    // the whole algae system is really volitile in software but if it works it works
    // also who even cares about this if we cant score barge
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmRestingAngle);
    subsystems.algaeIntake.setVoltage(AlgaeConstants.kRestVoltage);
  }

  @Override
  public boolean isFinished() {
    return !subsystems.algaeIntake.hasPiece();
  }
}
