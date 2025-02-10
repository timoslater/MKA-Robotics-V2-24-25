package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class RotatingArmTest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;

    private final double ticks = 1425.1; // for 435 rpm motors
    private DcMotorEx test;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        test = hardwareMap.get(DcMotorEx.class, "testMotor");
        //test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //test.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();



        while (!isStopRequested() && opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPosition = test.getCurrentPosition();
            double pid = controller.calculate(armPosition,target);
            double ff = Math.cos(Math.toRadians(target/ticks)) * f;

            double power = pid + ff;

            test.setPower(power);

            telemetry.addData("power", power);
            telemetry.addData("position", armPosition);
            telemetry.addData("target", target);
            telemetry.update();
        }



    }
}
