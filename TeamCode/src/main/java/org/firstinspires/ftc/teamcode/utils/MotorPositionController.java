package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorPositionController {
    private PIDController pid = new PIDController(0, 0,0);
    private double p = 0, i = 0, d = 0;
    private double syncP = 0, syncI = 0, syncD = 0;
    private double ff = 0;
    private int target = 0;
    private final double ticks = 384.5; // for 435 rpm motors
    private final double maxPower = 0.9;
    public DcMotorEx motor1, motor2;
    private MotorSyncController syncController;
    private double motorPower1, motorPower2;
    private double syncValue;
    public boolean enabled = true;

    public MotorPositionController(DcMotorEx motor1, DcMotorEx motor2, double p, double i, double d, double f, double ticks, int target) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        updatePositionValues(p, i, d, f, ticks, target);
    }

    public MotorPositionController(DcMotorEx motor1, DcMotorEx motor2, MotorSyncController syncController, double p, double i, double d, double f, double ticks, int target) {
        updatePositionValues(p, i, d, f, ticks, target);
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.syncController = syncController;
        syncController.updateValues(syncP, syncI, syncD);
    }

    public void update() {
        double calculatedPower = pid.calculate(motor1.getCurrentPosition(), target);
        motorPower1 = Math.min(calculatedPower, maxPower);
        motor1.setPower(motorPower1);
        if (motor2 != null) {
            syncValue = syncController.getSyncValue(motor2.getVelocity(), motor1.getVelocity());
            motorPower2 = motorPower1 + syncValue;
            motor2.setPower(motorPower2);
        }
    }

    public boolean isBusy() {
        return Math.abs(target - motor1.getCurrentPosition()) > 20;
    }
    public double getMotorPower1() { return motorPower1; }

    public double getMotorPower2() { return motorPower2; }

    public void setTarget(int target) {
        this.target = target;
    }

    public void updatePositionValues(double p, double i, double d, double f, double ticks, int target) {
        this.pid.setPID(p, i, d);
        this.ff = Math.cos(Math.toRadians(target/ticks)) * f;
        this.target = target;
    }

    public void updateSyncValues(double p, double i, double d) {
        this.syncController.updateValues(p, i, d);
    }
}
