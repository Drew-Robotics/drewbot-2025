// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.AlgaeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeCommand extends Command {
  /** Creates a new IntakeAlgaeCommand. */
  public IntakeAlgaeCommand() {
    addRequirements(
      subsystems.algaeIntake, 
      subsystems.algaeArm, 
      subsystems.algaeSensor
    );
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystems.algaeArm.setDesiredAngle(AlgaeConstants.kArmRestingAngle);
  }

  @Override
  public boolean isFinished() {
    return subsystems.algaeSensor.detecting();
  }
}
