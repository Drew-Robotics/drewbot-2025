package frc.lib.ledTools.finals;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDColor extends Color {

  public final int r;
  public final int g;
  public final int b;
  public final int a;

  public LEDColor(){
    this(0, 0, 0, 0);
  }

  public LEDColor(int r, int g, int b){
    this(r, g, b, 255);
  }

  public LEDColor(Color color){
    this(
      (int) (color.red * 255),
      (int) (color.green * 255),
      (int) (color.blue * 255)
    );
  }

  public LEDColor(int r, int g, int b, int a){
    this.r = clamp(r);
    this.g = clamp(g);
    this.b = clamp(b);
    this.a = clamp(a);
  }

  private int clamp(int num){
    return Math.min(Math.max(num, 0), 255);
  }

  public LEDColor add(LEDColor other) { // overlay other if needed
    double aThis = (double) a / 255d;
    double aOther = (double) other.a / 255d;

    double aOut = aOther + aThis * (1 - aOther); // thanks chatgpt

    // bruh more chatgpt
    LEDColor returnColor = new LEDColor(
      ((int) (other.r * aOther) + (int) (r * aThis * (1 - aOther))) / (int) aOut,
      ((int) (other.g * aOther) + (int) (g * aThis * (1 - aOther))) / (int) aOut,
      ((int) (other.b * aOther) + (int) (b * aThis * (1 - aOther))) / (int) aOut,
      (int) (aOut * 255)
    );

    // System.out.println("base : " + this + " other : " + other + " result : " + returnColor);

    return returnColor;
  }

  @Override
  public String toString() {
    return r + " " + g +  " " + b;
  }
}
