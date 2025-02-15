package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class ServoTest extends LinearOpMode {
    private ServoImplEx test1;
    private boolean running = false;
    public static double target;
    public static boolean reverse;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        test1 = hardwareMap.get(ServoImplEx.class, "testServo");

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (reverse) {
                test1.setDirection(Servo.Direction.REVERSE);
            } else {
                test1.setDirection(Servo.Direction.FORWARD);
            }


            test1.setPosition(target);
            telemetry.update();
        }



    }
}
