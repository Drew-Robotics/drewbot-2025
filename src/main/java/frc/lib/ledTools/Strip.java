package frc.lib.ledTools;

import com.ctre.phoenix.led.CANdle;

import frc.lib.ledTools.finals.LEDColor;
import frc.lib.ledTools.finals.StripState;

public class Strip {

  private CANdle m_candle;
  public final int startIndex, resolution;


  public Strip(CANdle candle, int startIndex, int resolution) {
    m_candle = candle;
    this.startIndex = startIndex;
    this.resolution = resolution;
  }

  public void setColor(LEDColor color) {
    m_candle.setLEDs(color.r, color.g, color.b);
  }

  // public void setStripState(StripState state) {
  //   state = state.scale(resolution);

  //   int ledIndex = startIndex;

  //   for (LEDColor color : state.colors){
  //     // System.out.println(color);
  //     m_candle.setLEDs(
  //       color.r, color.g, color.b, 
  //       0, ledIndex, 1
  //     );
  //     ledIndex++;
  //   }
  // }

}
