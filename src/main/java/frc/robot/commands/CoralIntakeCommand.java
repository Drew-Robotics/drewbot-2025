package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.CoralStates;
import frc.robot.subsystems.coral.CoralState;
import frc.robot.subsystems.coral.CoralIntakeSubsystem.CoralIntakeState;

public class CoralIntakeCommand extends Command {

  public CoralIntakeCommand() {
    addRequirements(subsystems.coralIntake);
  }

  @Override
  public void initialize() {
    // System.out.println("set coral instake intake");
    subsystems.coralIntake.setState(CoralIntakeState.Intake);
    new SetCoralStateCommand(CoralStates.kStation)
      .schedule();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.coralIntake.setState(CoralIntakeState.Hold);
    new SetCoralStateCommand(CoralStates.kRest).schedule();
  }

  @Override
  public boolean isFinished() {
    // System.out.println("ended " + subsystems.coralIntake.hasPiece());
    return subsystems.coralIntake.hasPiece() || 
      subsystems.elevator.getState() != CoralStates.kStation ||
      subsystems.coralArm.getState() != CoralStates.kStation;
  }
}
