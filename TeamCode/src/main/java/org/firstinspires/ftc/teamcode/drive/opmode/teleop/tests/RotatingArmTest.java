package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
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

    public static double p = 0.000, i = 0, d = 0.000, f = 0;
    public static double target = 0;
    public static boolean powered = false;
    private CRServo elbow1, elbow2;
    private AnalogInput encoder;
    public static double pow = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        elbow1 = hardwareMap.get(CRServo.class, "elbow1");
        elbow1.setDirection(DcMotorSimple.Direction.REVERSE);

        elbow2 = hardwareMap.get(CRServo.class, "elbow2");

        encoder = hardwareMap.get(AnalogInput.class, "servoEncoder");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();



        while (!isStopRequested() && opModeIsActive()) {
            if (powered) {
                elbow2.setPower(pow);
            }
//            } else {
                double elbowPosition = encoder.getVoltage() / 3.3 * 360;
//                controller.setPID(p, i, d);
//                double pid = controller.calculate(elbowPosition,target);
//
//                double ff;
//
//                if (elbowPosition > 200) {
//                    ff = -f;
//                } else if (elbowPosition < 100) {
//                    ff = f;
//                } else {
//                    ff = 0;
//                }
//                double power = pid + ff;
//
//                if (elbowPosition < 200) {
//                    elbow1.setDirection(DcMotorSimple.Direction.REVERSE);
//                    elbow2.setDirection(DcMotorSimple.Direction.FORWARD);
//                } else {
//                    elbow1.setDirection(DcMotorSimple.Direction.FORWARD);
//                    elbow2.setDirection(DcMotorSimple.Direction.REVERSE);
//                }
//
//                elbow1.setPower(power);
//                elbow2.setPower(power);

                telemetry.addData("position", elbowPosition);
                //telemetry.addData("power", power);
         //   }

//            controller.setPID(p, i, d);



//            telemetry.addData("power", pid);

            telemetry.addData("target", target);
            telemetry.update();
        }



    }
}
