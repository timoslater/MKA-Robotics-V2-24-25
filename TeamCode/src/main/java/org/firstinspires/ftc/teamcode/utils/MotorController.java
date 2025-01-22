package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorController {
    private PIDController pid;
    private double p = 0, i = 0, d = 0;
    private double ff = 0;
    private int target = 0;
    private final double ticks = 384.5; // for 435 rpm motors
    public DcMotor motor1, motor2;
    public boolean enabled = true;

    public MotorController(DcMotor motor1, DcMotor motor2, double p, double i, double d, double f, int target) {
        this.pid = new PIDController(p, i, d);
        this.ff = Math.cos(Math.toRadians(target/ticks)) * f;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.target = target;
    }

    public MotorController(DcMotor motor1, double p, double i, double d, double f, int target) {
        this(motor1, null, p, i, d, f, target);
    }

    public void update() {
        double power = pid.calculate(motor1.getCurrentPosition(), target);
        motor1.setPower(power);
        if (motor2 != null) {
            motor2.setPower(power);
        }
    }

    public boolean isBusy() {
        return Math.abs(target - motor1.getCurrentPosition()) > 50;
    }

    public void setTarget(int target) {
        this.target = target;
    }
}
