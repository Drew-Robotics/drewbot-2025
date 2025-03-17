package frc.robot.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller extends CommandXboxController {

  public Controller(int port) {
    super(port);
  }

  public void setRumble(double strength) {
    this.getHID().setRumble(RumbleType.kBothRumble, strength);
  }

  public InstantCommand rumbleCommand(double strength) {
    System.out.println("rumble " + strength);
    return new InstantCommand(
      () -> this.getHID().setRumble(RumbleType.kBothRumble, strength)
    );
  }

  public FunctionalCommand rumbleInteruptCommand(double strength) {
    return new FunctionalCommand(
      // init
      () -> setRumble(strength),
      // execute
      () -> {return;},
      // end
      interrupted -> setRumble(0),
      // finished
      () -> false
    );
  }

  public FunctionalCommand rumbleSupplierCommand(double strength, BooleanSupplier isFinishedSup) {
    return new FunctionalCommand(
      // init
      () -> setRumble(strength),
      // execute
      () -> {return;},
      // end
      interrupted -> setRumble(0),
      // finished
      isFinishedSup
    );
  }
}
