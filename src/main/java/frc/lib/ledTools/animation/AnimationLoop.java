package frc.lib.ledTools.animation;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.ledTools.Strip;
import frc.lib.ledTools.finals.LEDColor;
import frc.lib.ledTools.finals.StripState;

public abstract class AnimationLoop extends Animation{
  
  public static final double kDefaultSpeed = 10;

  private double m_secPerCycle;
  private int m_targetRes;

  public abstract LEDColor getLEDColor(double timeProgress, double ledProgress, Strip stripOut);

  public AnimationLoop(double secPerCycle, int targetRes) {
    super(targetRes);
    m_secPerCycle = secPerCycle;
  }

  public void setSpeed(double secPerCycle) {
    m_secPerCycle = secPerCycle;
  }

  // no longer needed since getStrip is overriden to use the new abstract method
  // its a little messy but idk what else to do and it should be fine
  @Override
  public LEDColor getLEDColor(double ledProgress, Strip stripOut) {
    return null;
  }

  @Override
  public StripState getStrip(Strip stripOut) {
    LEDColor[] colors = new LEDColor[stripOut.resolution];

    for (int ledIndex = 0; ledIndex < m_targetRes; ledIndex++){
      colors[ledIndex] = getLEDColor(
          (Timer.getFPGATimestamp() % m_secPerCycle) / m_secPerCycle,
          (double) ledIndex / (double) stripOut.resolution,
          stripOut
      );
    }
    return new StripState(colors);
  }
}
