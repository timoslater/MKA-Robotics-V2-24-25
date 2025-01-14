package org.firstinspires.ftc.teamcode.drive.opmode.teleop.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.TeleOp2024;

public class AsyncArmActions {
    private boolean liftRunning;
    private int targetPos;
    private int lastPosition;
    private int pos;
    private double power;
    private double defaultPower;
    private DcMotor lift1, lift2;
    private TeleOp2024 teleop;

    public AsyncArmActions(double defaultPower, TeleOp2024 teleop) {
        this.liftRunning = teleop.liftRunning;
        this.teleop = teleop;
        this.lastPosition = 0;
        this.defaultPower = defaultPower;
        this.power = defaultPower;
    }

    public void moveLiftTo(int targetPos, double power) {
        this.power = power;
        moveLiftTo(targetPos);
    }
    public void moveLiftTo(int targetPos) {
        if (Math.abs(targetPos - lastPosition) > 50) {
            this.targetPos = targetPos;

            if (!liftRunning) {
                this.power = targetPos > lift1.getCurrentPosition() ? this.power : -this.power;

                lift1.setPower(power);
                lift2.setPower(power);

                liftRunning = true;
            }

            int pos = lift1.getCurrentPosition();
            telemetry.addData("liftPos", pos);
            telemetry.update();

            if (Math.abs(targetPos - pos) < 50) {
                lift1.setPower(0);
                lift2.setPower(0);
                liftRunning = false;
                this.lastPosition = pos;
            }
        }
        teleop.liftRunning = this.liftRunning;
    }
}
