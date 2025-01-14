package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Pitching Arm Test", group = "Linear Opmode")
public class PitchingArmTest extends LinearOpMode {

    private Servo wristX = null;
    private Servo wristY = null;
    private AnalogInput analog1 = null;
    private AnalogInput analog2 = null;


    public void rotateForward(Servo servo) {
        servo.setPosition(servo.getPosition()+.02);
    }
    public void rotateBackward(Servo servo) {
        servo.setPosition(servo.getPosition()-.02);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized", "haggis");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        wristX = hardwareMap.get(Servo.class, "servo1");
        wristY = hardwareMap.get(Servo.class, "servo2");

        analog1 = hardwareMap.get(AnalogInput.class, "analog1");
        analog2 = hardwareMap.get(AnalogInput.class, "analog2");



        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                rotateBackward(wristX);
            } else if (gamepad1.right_bumper) {
                rotateForward(wristX);
            }

            if (gamepad1.left_trigger > 0.1) {
                rotateBackward(wristY);
            } else if (gamepad1.right_trigger > 0.1) {
                rotateForward(wristY);
            }

            telemetry.addData("Position 1", analog1.getVoltage() / 3.3 * 360);
            telemetry.addData("Position 2", analog2.getVoltage() / 3.3 * 360);
            telemetry.update();
        }
    }
}