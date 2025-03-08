package frc.robot.constants;

import java.util.List;

import frc.robot.subsystems.drive.ReefSide;

public class ReefSides {
    public static final ReefSide kFront = new ReefSide(18);
    public static final ReefSide kFrontLeft = new ReefSide(0);
    public static final ReefSide kFrontRight = new ReefSide(0);

    public static final ReefSide kLeft = new ReefSide(0);
    public static final ReefSide kRight = new ReefSide(0);

    public static final ReefSide kBack = new ReefSide(0);
    public static final ReefSide kBackLeft = new ReefSide(0);
    public static final ReefSide kBackRight = new ReefSide(0);


    public static final List<ReefSide> kReefSides = List.of(
        kFront, kFrontLeft, kFrontRight,
        kLeft, kRight,
        kBack, kBackLeft, kBackRight
    );
}
