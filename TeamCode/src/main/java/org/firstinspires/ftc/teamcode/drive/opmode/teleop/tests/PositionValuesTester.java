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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.AxonServo;
import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;

@Config
@TeleOp(name = "Position Values Tester", group = "Linear Opmode")
public class PositionValuesTester extends LinearOpMode {
    private DcMotorEx lift, slide1, slide2;
    private ServoImplEx elbow1, elbow2, wrist, rotate, grab;
    private MotorPositionController liftController, slideController;
    public static int liftTargetPosition, slideTargetPosition;
    public static double elbowPosition, wristPosition, rotatePosition, grabPosition;
    public static boolean movementDisabled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
//        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
//
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        elbow1 = hardwareMap.get(ServoImplEx.class, "elbow1");
        elbow1.setDirection(Servo.Direction.REVERSE);

        elbow2 = hardwareMap.get(ServoImplEx.class, "elbow2");

        wrist = hardwareMap.get(ServoImplEx.class, "wrist");

        rotate = hardwareMap.get(ServoImplEx.class, "rotate");

        grab = hardwareMap.get(ServoImplEx.class, "grab");

        liftController = new MotorPositionController(lift, null,0.002, 0, 0.0003, 0.1, 384.5, 0);
        slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
//            if (movementDisabled) {
//                elbow1.deactivate();
//                elbow2.deactivate();
//                wrist.deactivate();
//                rotate.deactivate();
//                grab.deactivate();
//
//                telemetry.addLine("Enable/Disable Movement By Changing \"movementDisabled\" (Set Position Values First!)");
//
//            } else {
//                elbow1.activate();
//                elbow2.activate();
//                wrist.activate();
//                rotate.activate();
//                grab.activate();
            elbow1.setPosition(elbowPosition);
            elbow2.setPosition(elbowPosition);

            wrist.setPosition(wristPosition);

            rotate.setPosition(rotatePosition);

            grab.setPosition(grabPosition);
//
                liftController.setTarget(liftTargetPosition);
                liftController.update();
//
                slideController.setTarget(slideTargetPosition);
                slideController.update();


//
//                try {
//                    telemetry.addData("elbow1 servo position", elbow1.getPosition());
//                    telemetry.addData("elbow2 servo position", elbow2.getPosition());
//                    telemetry.addData("wrist servo position", wrist.getPosition());
//                    telemetry.addData("rotate servo position", rotate.getPosition());
//                    telemetry.addData("grab servo position", grab.getPosition());
////
//                    telemetry.addData("lift motor position", lift.getCurrentPosition());
////                    telemetry.addData("lift2 motor position", lift2.getCurrentPosition());
////                    telemetry.addData("slide1 motor position", slide1.getCurrentPosition());
////                    telemetry.addData("slide2 motor position", slide2.getCurrentPosition());
//
//                    telemetry.update();
//
//                } catch (Exception e) {
//                    throw new RuntimeException(e);
//                }

        }
    }
}
