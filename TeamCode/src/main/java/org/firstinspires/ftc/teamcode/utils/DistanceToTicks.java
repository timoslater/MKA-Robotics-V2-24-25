package org.firstinspires.ftc.teamcode.utils;

public class DistanceToTicks {
    private static final double slideMotorRadius = 1, slideMotorPPR = 384.5;
    public static int forSlide(double distance) {
        return (int) Math.round(distance * slideMotorPPR / (2*Math.PI*slideMotorRadius));
    }

    /*

    Radius needs to be tested
    Current PPR is for 435 RPM Motors
    Units for radius and distance must be the same
    Add more motors if needed

    */
}
