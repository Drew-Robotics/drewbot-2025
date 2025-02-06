package lib;

import java.util.Arrays;

public class LibFuncs {
    public static double sum(double... nums) {
        return Arrays.stream(nums).reduce(0, (a, b) -> a + b);
    }

    public static double mult(double... nums) {
        return Arrays.stream(nums).reduce(1, (a, b) -> a * b);
    }
}
