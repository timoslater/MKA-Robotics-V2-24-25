package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

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
@TeleOp(group = "drive")
public class ServoTest extends LinearOpMode {
    private ServoImplEx test = null;
    private boolean running = false;

    private void setPosControlled(Servo servo, double endPos, double step) {



        double startPos = servo.getPosition();
        int steps = (int) (Math.abs(startPos - endPos)/step);
        int direction = startPos - endPos > 0 ? -1 : 1;

        double currentPos = startPos;



        for (int i = 0; i < steps; i++) {
            running = true;
            currentPos += step * direction;
            servo.setPosition(currentPos);
            sleep(500/steps);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        test = hardwareMap.get(ServoImplEx.class, "testServo");



        waitForStart();

        double startPos = test.getPosition();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.dpad_left) {
                test.setPosition(test.getPosition()+.001);
            }
            else if (gamepad1.dpad_right && !running) {
                test.setPosition(test.getPosition()-.001);
            }

            telemetry.addData("Servo Position", test.getPosition());
            telemetry.update();
        }



    }
}
