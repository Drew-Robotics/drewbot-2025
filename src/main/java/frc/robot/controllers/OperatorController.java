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

  public Trigger getMoveNearestRightCoral() {
    return rightBumper();
  }

  public Trigger getMoveNearestLeftCoral() {
    return leftBumper();
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

  public Trigger getScore() {
    return rightTrigger();
  }

  public Trigger getRemoveAlgae() {
    return leftBumper();
  }

  public Trigger getToggleExtendAlgae() {
    return null; // dpad left
  }

  public Trigger getPickupAlgae() {
    return null; // dpad down
  }

  public Trigger getDropAlgae() {
    return null; // dpad right
  }
}
