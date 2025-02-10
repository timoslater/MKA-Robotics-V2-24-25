package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class AxonPositionTest extends LinearOpMode {
    private AnalogInput servoReader1, servoReader2;

    @Override
    public void runOpMode() throws InterruptedException {

        servoReader1 = hardwareMap.get(AnalogInput.class, "servoReader1");
        servoReader2 = hardwareMap.get(AnalogInput.class, "servoReader2");


        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("servo 1", servoReader1.getVoltage() / 3.3 * 360);
            telemetry.addData("servo 2", servoReader2.getVoltage() / 3.3 * 360);
            telemetry.update();
        }



    }
}
