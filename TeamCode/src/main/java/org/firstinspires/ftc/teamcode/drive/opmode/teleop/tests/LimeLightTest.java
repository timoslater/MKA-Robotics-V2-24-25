package org.firstinspires.ftc.teamcode.drive.opmode.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;

@Config
@TeleOp
public class LimeLightTest extends LinearOpMode {
    private boolean grabbing = false;
    private boolean clawClosed = false;
    private Limelight3A limelight;
    private Servo rotate;

    private int[] pipelines = {0,1,2};
    private int pipelineIndex = 0;

    private Servo grab = null;
    private Servo wrist;
    private Servo elbow1;
    private Servo elbow2;
    private MotorPositionController slideController;


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    //private DcMotor revArm = null;
    private DcMotor rightRear = null;

    private DcMotorEx slide1 = null;
    private DcMotorEx slide2 = null;
    private Gamepad driveGamepad = null;


    public static double offset = 0.2;


    public void movement() {
        double modifier = 1;//nearBoard ? 0.65 : 1;
        double Lpower = 1*modifier;
        double Rpower = .517; //0.52*modifier;//*modifier;
        boolean reverseStick = true;

        double r = Lpower * Math.hypot((!reverseStick) ? driveGamepad.left_stick_x : driveGamepad.right_stick_x, (!reverseStick) ? -driveGamepad.left_stick_y : -driveGamepad.right_stick_y);
        double robotAngle = Math.atan2((!reverseStick) ? -driveGamepad.left_stick_y : -driveGamepad.right_stick_y, (!reverseStick) ? driveGamepad.left_stick_x : driveGamepad.right_stick_x) + 3 * Math.PI / 4;
        double rightX = Rpower * ((!reverseStick) ? driveGamepad.right_stick_x : driveGamepad.left_stick_x) * 1;
        double rightY = Rpower * ((!reverseStick) ? driveGamepad.right_stick_y : driveGamepad.left_stick_y) * 1;
        double v1 = r * Math.cos(robotAngle) - rightX + rightY;
        double v2 = r * Math.sin(robotAngle) + rightX + rightY;
        double v3 = r * Math.sin(robotAngle) - rightX + rightY;
        double v4 = r * Math.cos(robotAngle) + rightX + rightY;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }

    public void slideUp() {
        slide1.setPower(1);
        slide2.setPower(1);
    }
    public void slideDown() {
        slide1.setPower(-1);
        slide2.setPower(-1);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        elbow1 = hardwareMap.get(ServoImplEx.class, "elbow1");
        elbow1.setDirection(Servo.Direction.REVERSE);
        elbow2 = hardwareMap.get(ServoImplEx.class, "elbow2");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);


        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);

        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0,0,0), 0.015, 0, 0.0005, 0.5, 384.5, 0);

        grab = hardwareMap.get(Servo.class, "grab");
        rotate = hardwareMap.get(Servo.class, "rotate");
        elbow1 = hardwareMap.get(ServoImplEx.class, "elbow1");
        elbow1.setDirection(Servo.Direction.REVERSE);
        elbow2 = hardwareMap.get(ServoImplEx.class, "elbow2");
        wrist = hardwareMap.get(Servo.class, "wrist");
        driveGamepad = gamepad1;

        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(0);


        rotate = hardwareMap.get(Servo.class, "rotate");
        rotate.setDirection(Servo.Direction.REVERSE);
        /*
         * Starts polling for data.
         */

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        waitForStart();
        limelight.start();
        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            movement();
            if (grabbing) {
                elbow1.setPosition(0.5);
                elbow2.setPosition(0.5);
                wrist.setPosition(1);
            } else {
                if (result != null && !clawClosed) {

                    double[] outputs = result.getPythonOutput();

                    telemetry.addData("Angle", outputs[5]);
                    rotate.setPosition((outputs[5]/180)+offset);
                    telemetry.addData("Valid", result.isValid());
                }
                elbow1.setPosition(0.6); // hover
                elbow2.setPosition(0.6);
                wrist.setPosition(1);
            }
            if (driveGamepad.triangle) {
                clawClosed = false;
                grab.setPosition(0.4);
            }

            if (driveGamepad.cross) {
                clawClosed = true;
                grab.setPosition(0.75);
            }

            if (driveGamepad.square) {
                offset+=0.01;
            }

            if (driveGamepad.circle) {

                offset-=0.01;
            }

            if(driveGamepad.dpad_right){
                pipelineIndex++;
                limelight.pipelineSwitch(pipelines[pipelineIndex%3]);
            }

            if (driveGamepad.left_stick_button) {
                grabbing = true;
            } else if (driveGamepad.right_stick_button) {
                grabbing = false;
            }

            if (driveGamepad.left_trigger > 0 && slide1.getCurrentPosition() < 2500) {
                double slidePower = driveGamepad.left_trigger;
                slide1.setPower(slidePower);
                slide2.setPower(slidePower);
            } else if (driveGamepad.right_trigger > 0 && slide1.getCurrentPosition() > 0) {
                double slidePower = driveGamepad.right_trigger;
                slide1.setPower(-slidePower);
                slide2.setPower(-slidePower);
            } else {
                slideController.setTarget(slide1.getCurrentPosition());
                slideController.update();
            }




            LLStatus status = limelight.getStatus();

            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());



            telemetry.update();
        }
    }
}