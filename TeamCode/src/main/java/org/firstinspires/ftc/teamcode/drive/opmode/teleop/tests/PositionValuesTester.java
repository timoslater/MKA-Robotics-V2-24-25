package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.AxonServo;
import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;

@Config
@TeleOp(name = "Position Values Tester", group = "Linear Opmode")
public class PositionValuesTester extends LinearOpMode {
    private DcMotorEx lift1, lift2, slide1, slide2;
    private AxonServo elbow1, elbow2, wrist, rotate, grab;
    private MotorPositionController liftController, slideController;
    public static int liftTargetPosition, slideTargetPosition;

    public static double elbowPosition, wristPosition, rotatePosition, grabPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        elbow1 = new AxonServo(hardwareMap.get(Servo.class, "elbow1"), hardwareMap.get(AnalogInput.class, "elbow1Input"));

        elbow2 = new AxonServo(hardwareMap.get(Servo.class, "elbow2"), hardwareMap.get(AnalogInput.class, "elbow2Input"));
        elbow2.setDirection(Servo.Direction.REVERSE);

        wrist = new AxonServo(hardwareMap.get(Servo.class, "wrist"), hardwareMap.get(AnalogInput.class, "wristInput"));

        rotate = new AxonServo(hardwareMap.get(Servo.class, "rotate"), hardwareMap.get(AnalogInput.class, "rotateInput"));

        grab = new AxonServo(hardwareMap.get(Servo.class, "grab"), hardwareMap.get(AnalogInput.class, "grabInput"));

        liftController = new MotorPositionController(lift1, lift2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);
        slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            liftController.setTarget(liftTargetPosition);
            liftController.update();

            slideController.setTarget(slideTargetPosition);
            slideController.update();

            elbow1.setPosition(elbowPosition);
            elbow2.setPosition(elbowPosition);

            wrist.setPosition(wristPosition);

            rotate.setPosition(rotatePosition);

            grab.setPosition(grabPosition);
        }
    }
}
