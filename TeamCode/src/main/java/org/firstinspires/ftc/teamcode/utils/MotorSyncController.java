package org.firstinspires.ftc.teamcode.utils;

public class MotorSyncController {
    private PIDController pid = new PIDController(0, 0,0);
    private double p = 0, i = 0, d = 0;

    public MotorSyncController(double p, double i, double d) {
        updateValues(p, i, d);
    }

    public double getMultiplier(double actualVelocity, double targetVelocity) {
        double powerMultiplier = pid.calculate(actualVelocity, targetVelocity);

        return powerMultiplier;
    }

    public void updateValues(double p, double i, double d) {
        this.pid.setPID(p, i, d);
    }
}
