package frc.lib.ledTools.animation;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.ledTools.Strip;
import frc.lib.ledTools.finals.LEDColor;
import frc.lib.ledTools.finals.StripState;

public abstract class Animation {
  
  public static final double kDefaultSpeed = 10;

  private int m_targetRes;

  public abstract LEDColor getLEDColor(double ledProgress, Strip stripOut);

  public Animation(int targetRes) {
    m_targetRes = targetRes;
  }

  public void setTargetRes(int targetRes) {
    m_targetRes = targetRes;
  }

  public StripState getStrip(Strip stripOut) {
    LEDColor[] colors = new LEDColor[stripOut.resolution];

    for (int ledIndex = 0; ledIndex < m_targetRes; ledIndex++){
      colors[ledIndex] = getLEDColor(
          (double) ledIndex / (double) stripOut.resolution,
          stripOut
      );
    }
    return new StripState(colors);
  }
}
