package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IOConstants;

public class OperatorController extends Controller {
  public static OperatorController m_intance;

  public static OperatorController getInstance(){
    if (m_intance == null){
        m_intance = new OperatorController(IOConstants.OperatorController.kPort);
    }
    return m_intance;
  }

  public OperatorController(int port) {
    super(port);
  }

  public Trigger getSetStateRest() {
    return leftBumper();
  }

  public Trigger getSetStateStation() {
    return rightBumper();
  }

  public Trigger getSetStateL1() {
    return a();
  }

  public Trigger getSetStateL2() {
    return b(); 
  }

  public Trigger getSetStateL3() {
    return y(); 
  }

  public Trigger getSetStateL4() {
    return x(); 
  }

  public Trigger getCoralIntake() {
    return leftTrigger(0.5); // TODO: make this a constant
  }

  public Trigger getCoralOuttake() {
    return rightTrigger(0.5); // TODO: make this a constant
  }

  public Trigger getAlgaeIntake() {
    return povUp();
  }

  public Trigger getAlgaeOuttake() {
    return povDown();
  }
}
