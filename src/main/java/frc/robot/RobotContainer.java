// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//     .----.   @   @
//    / .-"-.`.  \v/
//    | | '\ \ \_/ )
//   ,-\ `-.' /.'  /
// '---`----'----'

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringTopic;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.CoralConstants.PID.CoralArm;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.algae.AlgaeArmSubsystem;
import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;
import frc.robot.subsystems.coral.CoralArmSubsystem;
import frc.robot.subsystems.coral.CoralIntakeSubsystem;
import frc.robot.subsystems.coral.ElevatorSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  
  private final SendableChooser<Command> autoChooser;

  public final class subsystems {
    public static final DriveSubsystem drive = DriveSubsystem.getInstance();
    public static final VisionSubsystem vision = VisionSubsystem.getInstance();

    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();

    public static final CoralArmSubsystem coralArm = CoralArmSubsystem.getInstance();
    public static final CoralIntakeSubsystem coralIntake = CoralIntakeSubsystem.getInstance();

    public static final AlgaeArmSubsystem algaeArm = AlgaeArmSubsystem.getInstance();
    public static final AlgaeIntakeSubsystem algaeIntake = AlgaeIntakeSubsystem.getInstance();
  }

  public final class controllers {
    public static final DriverController driver = DriverController.getInstance();
    public static final OperatorController operator = OperatorController.getInstance();
  }

  private static RobotContainer m_instance;
  public static RobotContainer getInstance() {
    if (m_instance == null)
      m_instance = new RobotContainer();
    return m_instance;
  }

  protected RobotContainer() {
    subsystems.drive.pathPlannerConfig();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoChooser.setDefaultOption("DefaultAuto", AutoBuilder.buildAuto("DefaultAuto"));

    configureBindings();
  }

  private void configureBindings() {
    subsystems.drive.setDefaultCommand(
      new DriveCommand(
        controllers.driver::getXVelocity, 
        controllers.driver::getYVelocity, 
        controllers.driver::getRotationalVelocity)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public String getAutonomousName() {
    StringTopic activeAutoTopic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Chooser/active");
    String activeAutoName = activeAutoTopic.getEntry("Super Cool Fast Awesome Auto", PubSubOption.sendAll(false)).get();
  
    return activeAutoName;
  }
}
