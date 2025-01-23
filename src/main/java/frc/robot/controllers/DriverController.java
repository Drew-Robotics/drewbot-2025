package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;
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
    return MathUtil.applyDeadband(getLeftY(), IOConstants.DriverController.kDeadband);
  }

  public double getYVelocity(){
    return MathUtil.applyDeadband(getLeftX(), IOConstants.DriverController.kDeadband);
  }

  public double getRotationalVelocity(){
    return MathUtil.applyDeadband(getRightX(), IOConstants.DriverController.kDeadband);
  }

  public Trigger getTurnToZeroButton(){
    return y();
  }
}