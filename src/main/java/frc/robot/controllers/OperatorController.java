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

  public double getNotePipelineMPS() {
    return getRightTriggerAxis();
  }

  public Trigger getRevingTrigger() {
    return rightBumper();
  }

  public Trigger getEjectTrigger() {
    return x();
  }

  public Trigger getFeedingShooter() {
    return a();
  }

  public Trigger getIntakeTrigger() {
    return b();
  }
}
