package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.IOConstants;

public class DriverController extends Controller{

  public static DriverController m_intance;

  public static DriverController getInstance(){
    if (m_intance == null){
        m_intance = new DriverController(IOConstants.DriverController.kPort);
    }
    return m_intance;
  }

  private DriverController(int port){
    super(port);
  }

  public double getXVelocity(){
    return MathUtil.applyDeadband(-getLeftY(), IOConstants.DriverController.kDeadband);
  }

  public double getYVelocity(){
    return MathUtil.applyDeadband(-getLeftX(), IOConstants.DriverController.kDeadband);
  }

  public double getRotationalVelocity(){
    return MathUtil.applyDeadband(-getRightX(), IOConstants.DriverController.kDeadband);
  }


  // fowards
  public Trigger getTurnTo0Button(){
    return y();
  }

  // left
  public Trigger getTurnTo90Button(){
    return x();
  }

  // back
  public Trigger getTurnTo180Button(){
    return a();
  }

  // right
  public Trigger getTurnTo270Button(){
    return b();
  }

  // front left -30
  public Trigger getTurnToFrontLeftSide() {
    return b().and(y());
  }
  // front right 30
  public Trigger getTurnToFrontRightSide() {
    return x().and(y());
  }
  // back left 180 -30
  public Trigger getTurnToBackLeftSide() {
    return a().and(b());
  }
  // back right 180 +30
  public Trigger getTurnToBackRightSide() {
    return a().and(x());
  }


  public Trigger getToggleAutoRotate() {
    return start();
  }

  public Trigger getScore() {
    return rightTrigger(0.5);
  }

  public Trigger getScoreSetup() {
    return leftTrigger(0.5);
  }

  public Trigger getRemoveAlgae() {
    return leftBumper();
  }

  public Trigger getSetStateStation() {
    return rightBumper();
  }
  
  // public Trigger getMoveNearestRightCoral() {
  //   return rightBumper();
  // }

  // public Trigger getMoveNearestLeftCoral() {
  //   return leftBumper();
  // }

}