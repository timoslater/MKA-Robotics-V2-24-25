package org.firstinspires.ftc.teamcode.drive.opmode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MotorController;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
@TeleOp(name = "MAIN", group = "Linear Opmode")
public class TeleOp2024 extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    //private DcMotor revArm = null;
    private DcMotor rightRear = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor slide = null;
    private DcMotor specimen = null;
    private Servo claw = null;
    private Servo rotate = null;
    private Servo clawSpecimen = null;
    private boolean resetting = false;
    private Gamepad driveGamepad = null;
    private Gamepad armGamepad = null;
    private IMU imu;
    private Servo hand;
    private Servo elbow;
    private int lastPos = 325;
    private boolean isDropping = false;
    private int rotateIndex;
    private double[] rotatePositions = {.055,.222, .555,.888};

    private MotorController liftController;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks = 384.5;

    public boolean liftRunning = false;
    public boolean positionSet = false;
    private double minLiftPower = 0.25;

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

    public void liftUp(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }
    public void liftDown(double power) {
        lift1.setPower(-power);
        lift2.setPower(-power);
    }

    public void slideUp() {
        slide.setPower(1);
    }
    public void slideDown() { slide.setPower(-1); }

    public void specimenUp(){
        specimen.setPower(-0.75);
    }
    public void specimenDown(){
        specimen.setPower(0.75);
    }

    public void clawOpen() {
        claw.setPosition(.3);
        if (!isDropping) {
            elbow.setPosition(0.296);
            hand.setPosition(0.416);
        }

    }

    public void clawClose() {
        claw.setPosition(.64);
        if (!isDropping) {
            elbow.setPosition(0.38);
            hand.setPosition(0.5);
        };
    }

    public void claw2Open(){
        clawSpecimen.setPosition(.3);
    }
    public void claw2Close(){
        clawSpecimen.setPosition(.64);
    }

    public void rotateClawR() {

        if(rotateIndex<rotatePositions.length-1) {
            rotateIndex++;
            rotate.setPosition(rotatePositions[rotateIndex]);
        }


    }
    public void rotateClawL() {
        if(rotateIndex>0) {
            rotateIndex--;
            rotate.setPosition(rotatePositions[rotateIndex]);
        }

    }

    public void armPositionIdle() {
        isDropping = false;
        hand.setPosition(.92);
        clawOpen();
    }

    public void armPositionDrop() throws InterruptedException {
        isDropping = true;
        elbow.setPosition(0);
        Thread.sleep(500);
        hand.setPosition(0);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized", "haggis");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        lift1 = hardwareMap.get(DcMotor.class, "lift");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        specimen = hardwareMap.get(DcMotor.class, "specimen");
        specimen.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "grabber");
        rotate = hardwareMap.get(Servo.class,"rotator");
        clawSpecimen = hardwareMap.get(Servo.class, "clawSpecimen");
        elbow = hardwareMap.get(Servo.class, "elbow");
        hand = hardwareMap.get(Servo.class, "hand");

        //AsyncArmActions armControl = new AsyncArmActions(0.25, this);
        Gamepad currDriveGamepad = new Gamepad();
        Gamepad currArmGamepad = new Gamepad();

        Gamepad prevDriveGamepad = new Gamepad();
        Gamepad prevArmGamepad = new Gamepad();

        driveGamepad = gamepad1;

        liftController = new MotorController(lift1, lift2, 0.009, 0, 0.0001, 0.05, lastPos);


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //armControl.moveLiftTo(2400);

            if (!positionSet && lift1.getCurrentPosition() > 250) {
                positionSet = true;
                armPositionIdle();
            }

            movement();

            armGamepad = gamepad2.getGamepadId() == -1 ? gamepad1 : gamepad2;

            prevArmGamepad.copy(currArmGamepad);
            prevDriveGamepad.copy(currDriveGamepad);

            currArmGamepad.copy(armGamepad);
            currDriveGamepad.copy(driveGamepad);


            if (armGamepad.left_trigger > 0 && (lift1.getCurrentPosition() < 5900 || resetting)) {
                liftUp(armGamepad.left_trigger * 0.75);
                lastPos = lift1.getCurrentPosition();
            } else if (armGamepad.right_trigger > 0) { //&& (lift1.getCurrentPosition() > 230 || resetting)) {
                if (lift1.getCurrentPosition() < 400) {
                    liftDown(armGamepad.right_trigger * 0.75 * Math.max(Math.pow(minLiftPower, Math.abs(lift1.getCurrentPosition() - 0)), minLiftPower));
                    // f(x) = (total power) * (min power)^(right_trigger)
                } else {
                    liftDown(armGamepad.right_trigger * 0.75);
                }
                lastPos = lift1.getCurrentPosition();
            } else {
                liftController.setTarget(lastPos);
                liftController.update();
            }

            if (armGamepad.dpad_up) {
                armPositionDrop();
            }

            if (armGamepad.dpad_down) {
                armPositionIdle();
            }

            if (armGamepad.right_bumper && (slide.getCurrentPosition() > -2670  || resetting)) {
                slideUp();
            } else if (armGamepad.left_bumper) {
                slideDown();
            } else {
                slide.setPower(0);
            }

            if (driveGamepad.options && driveGamepad.share) {
                if (resetting) {
                    lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    resetting = false;
                } else {
                    resetting = true;
                }
            }

            if (armGamepad.triangle) {
                clawOpen();
            } else if (armGamepad.cross) {
                clawClose();
            }
            if (currArmGamepad.dpad_right && !prevArmGamepad.dpad_right){

                rotateClawR();
            } else if(currArmGamepad.dpad_left && !prevArmGamepad.dpad_left){
                rotateClawL();
            }

            if(armGamepad.right_stick_button){
                specimenUp();
            } else if(armGamepad.left_stick_button){
                specimenDown();
            } else{
                specimen.setPower(0);
            }

            if(armGamepad.square){
                claw2Open();
            } else if(armGamepad.circle){
                claw2Close();
            }

            telemetry.addData("Is Resetting?", resetting);
            telemetry.addData("Lift Position", lastPos);
            telemetry.addData("Target", target);
            telemetry.addData("rotate index", rotateIndex);
            telemetry.update();
        }
    }
}