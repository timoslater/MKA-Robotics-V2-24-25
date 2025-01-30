package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@TeleOp(group = "drive")
public class EncoderTest extends LinearOpMode {
    private ArrayList<DcMotor> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        motors = new ArrayList<DcMotor>() {{
            add(hardwareMap.get(DcMotor.class, "slide"));
            add(hardwareMap.get(DcMotor.class, "lift"));
            add(hardwareMap.get(DcMotor.class, "specimen"));

        }};



        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            for (DcMotor motor : motors) {
                telemetry.addData(motor.getConnectionInfo(), motor.getCurrentPosition());
            }

            if (gamepad1.touchpad) {
                for (DcMotor motor : motors) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            telemetry.update();
        }



    }
}
