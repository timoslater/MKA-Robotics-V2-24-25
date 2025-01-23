package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utils.MotorController;

@Config
@TeleOp(name = "Motor Controller Test", group = "Linear Opmode")
public class MotorControllerTest extends LinearOpMode {
    public static double p = 0, i = 0, d = 0, f = 0;

    public static int target = 0;

    public static double ticks = 384.5; // for 435 rpm motors

    public static boolean reverse = false;

    private DcMotorEx testMotor;
    private MotorController motorController;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        motorController = new MotorController(testMotor, p, i, d, f, ticks,0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if (reverse) {
                testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            motorController.setValues(testMotor, null, p, i, d, f, ticks, target);
            motorController.update();

            telemetry.addData("position", testMotor.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }


}
