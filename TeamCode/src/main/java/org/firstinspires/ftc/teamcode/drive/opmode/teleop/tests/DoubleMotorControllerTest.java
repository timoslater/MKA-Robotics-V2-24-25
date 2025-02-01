package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;

@Config
@TeleOp(name = "Double Motor Controller Test", group = "Linear Opmode")
public class DoubleMotorControllerTest extends LinearOpMode {
    public static double positionP = 0, positionI = 0, positionD = 0, positionF = 0;
    public static double syncP = 0, syncI = 0, syncD = 0;

    public static int target = 0;

    public static double ticks = 384.5; // for 435 rpm motors

    public static boolean reverse = false;

    private DcMotorEx testMotor1, testMotor2;
    private MotorPositionController positionController;

    @Override
    public void runOpMode() {
        testMotor1 = hardwareMap.get(DcMotorEx.class, "testMotor1");
        testMotor2 = hardwareMap.get(DcMotorEx.class, "testMotor2");

        positionController = new MotorPositionController(testMotor1, testMotor2, new MotorSyncController(syncP, syncI, syncD), positionP, positionI, positionD, positionF, ticks, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        testMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if (reverse) {
                testMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
                testMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                testMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                testMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            positionController.updatePositionValues(positionP, positionI, positionD, positionF, ticks, target);
            positionController.updateSyncValues(syncP, syncI, syncD);
            positionController.update();

            double motorVelocity1 = testMotor1.getVelocity();
            double motorVelocity2 = testMotor2.getVelocity();

            telemetry.addData("position", testMotor1.getCurrentPosition());
            telemetry.addData("motor1 (main) velocity", motorVelocity1);
            telemetry.addData("motor1 (main) power", positionController.getMotorPower1());
            telemetry.addData("motor2 (secondary) velocity", motorVelocity2);
            telemetry.addData("motor2 (secondary) power", positionController.getMotorPower2());
            telemetry.addData("velocity sync error", motorVelocity1 - motorVelocity2);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }


}
