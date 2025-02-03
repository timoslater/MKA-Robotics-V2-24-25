package org.firstinspires.ftc.teamcode.utils;

public class MotorSyncController {
    private PIDController pid = new PIDController(0, 0,0);
    private double p = 0, i = 0, d = 0;

    public MotorSyncController(double p, double i, double d) {
        updateValues(p, i, d);
    }

    public double getSyncValue(double actualVelocity, double targetVelocity) {
        return pid.calculate(actualVelocity, targetVelocity);
    }

    public void updateValues(double p, double i, double d) {
        this.pid.setPID(p, i, d);
    }
}
