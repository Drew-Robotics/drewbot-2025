package frc.lib.ledTools.finals;

import java.util.ArrayList;

public final class StripState {

  public final LEDColor[] colors;

  public StripState() {
    this(new LEDColor[]{new LEDColor()});
  }


  public StripState(int resolution) {
    this(new LEDColor[resolution]);
  }

  public StripState(LEDColor color) {
    this.colors = new LEDColor[]{color};
  }

  public StripState(LEDColor[] colors) {
    if (colors.length != 0){
      for (LEDColor color : colors) {
        if (color == null) color = new LEDColor();
      }
      
      this.colors = colors;
    } 
    else {
      this.colors = new LEDColor[]{new LEDColor()};
    }
  }

  public StripState scale(int resolution) {
    LEDColor[] colorsOut = new LEDColor[resolution];

    double step = (double) resolution / (double) colors.length;
    double indexDouble = 0;
    int indexInt = 0;

    for (int rescaledIndex = 0; rescaledIndex < colorsOut.length; rescaledIndex++) {
      indexInt = (int) Math.round(indexDouble);

      indexInt = Math.max(indexInt, 0);
      indexInt = Math.min(indexInt, colors.length - 1);

      colorsOut[rescaledIndex] = colors[indexInt];

      indexDouble += step;
    }

    return new StripState(colorsOut);
  }

  public static StripState add(ArrayList<StripState> states) {
    StripState returnState = new StripState();

    for (StripState state : states) {
      returnState.add(state);
    }

    return returnState;
  }

  public StripState add(StripState overlayState) {

    int resolution = Math.max(colors.length, overlayState.colors.length);
    

    StripState baseState = scale(resolution);
    overlayState = overlayState.scale(resolution);

    LEDColor[] colorsOut = new LEDColor[resolution];

    for (int ledIndex = 0; ledIndex < resolution; ledIndex++) {
      LEDColor base = baseState.colors[ledIndex];
      LEDColor overlay = overlayState.colors[ledIndex];

      if ((base == null) && (overlay == null)) colorsOut[ledIndex] = new LEDColor();
      if (base == null) colorsOut[ledIndex] = overlay;
      if (overlay == null) colorsOut[ledIndex] = base;
      else colorsOut[ledIndex] = base.add(overlay);
    }

    return new StripState(colorsOut);
  }
}
