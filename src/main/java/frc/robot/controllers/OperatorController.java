package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.OtherConstants;

public class OperatorController extends Controller {
  public static OperatorController m_intance;

  public static OperatorController getInstance(){
    if (m_intance == null){
        m_intance = new OperatorController(OtherConstants.OperatorController.kPort);
    }
    return m_intance;
  }

  public OperatorController(int port) {
    super(port);
  }

  public Trigger getSetStateL1() {
    return a();
  }

  public Trigger getSetStateL2() {
    return b(); 
  }

  public Trigger getSetStateL3() {
    return x(); 
  }

  public Trigger getSetStateL4() {
    return y(); 
  }

  public Trigger getSetStateRMAlgaeL2() {
    return b();
  }

  public Trigger getSetStateRMAlgaeL3() {
    return x();
  }

  public Trigger getSetBranchLeft() {
    return leftBumper();
  }

  public Trigger getSetBranchRight() {
    return rightBumper();

  }

  public Trigger getClimberUp() {
    return povUp();
  }

  public Trigger getClimberHold() {
    return povDown();
  }

  public Trigger getAlgaeIntake() {
    return leftTrigger(0.5);
  }

  public Trigger getAlgaeOuttake() {
    return rightTrigger(0.5);
  }

  public Trigger getStow() {
    return leftStick().or(rightStick());
  }

  public Trigger getDebugCoralEject() {
    return start();
  }

  public Trigger getDebugAlgaeIntakeBack() {
    return back();
  }
}
