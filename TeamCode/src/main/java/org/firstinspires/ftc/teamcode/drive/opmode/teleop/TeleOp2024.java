package org.firstinspires.ftc.teamcode.drive.opmode.teleop;


import android.telephony.AccessNetworkConstants;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;

@Config
@TeleOp(name = "MAIN", group = "Linear Opmode")
public class TeleOp2024 extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    //private DcMotor revArm = null;
    private DcMotor rightRear = null;
    private DcMotorEx lift = null;
    private DcMotorEx slide1 = null;
    private DcMotorEx slide2 = null;
    private DcMotor specimen = null;
    private Servo grab = null;
    private Servo rotate = null;
    private Servo clawSpecimen = null;
    private boolean resetting = false;
    private Gamepad driveGamepad = null;
    private Gamepad armGamepad = null;
    private IMU imu;
    private Servo hand;
    private Servo elbow1;
    private Servo elbow2;
    private Servo wrist;
    private int lastPos = 0;

    private boolean grabbing = false;

    private int lastSlidePos = 0;
    private boolean isDropping = false;
    private int rotateIndex;
    private double[] rotatePositions = {.055,.222, .555,.888};

    private MotorPositionController liftController, slideController;
    public int target = 0;
    private final double ticks = 384.5;

    public boolean liftRunning = false;
    public boolean positionSet = false;
    private double minLiftPower = 0.25;

    private long stateStartTime = 0;
    ElapsedTime time = new ElapsedTime();
    public int subState = 0;
    private boolean subStateDone = false;

    public enum RobotState {
        FLOOR_GRAB,
        SPECIMEN_GRAB,
        SPECIMEN_TRANSITION,
        SPECIMEN_DROP,
        HIGH_BASKET,
        LOW_BASKET,
        NEUTRAL,
    }

    public static RobotState robotState = RobotState.FLOOR_GRAB;

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
    public void rotateClawR() {
        rotate.setPosition(rotate.getPosition() - .01);
    }
    public void rotateClawL() {
            rotate.setPosition(rotate.getPosition() + .01);
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

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);

        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grab = hardwareMap.get(Servo.class, "grab");
        rotate = hardwareMap.get(Servo.class, "rotate");
        elbow1 = hardwareMap.get(ServoImplEx.class, "elbow1");
        elbow1.setDirection(Servo.Direction.REVERSE);
        elbow2 = hardwareMap.get(ServoImplEx.class, "elbow2");
        wrist = hardwareMap.get(Servo.class, "wrist");

//        //AsyncArmActions armControl = new AsyncArmActions(0.25, this);
        Gamepad currDriveGamepad = new Gamepad();
        Gamepad currArmGamepad = new Gamepad();

        Gamepad prevDriveGamepad = new Gamepad();
        Gamepad prevArmGamepad = new Gamepad();

        driveGamepad = gamepad1;
        armGamepad = gamepad1;

        liftController = new MotorPositionController(lift, null,0.002, 0, 0.0003, 0.1, 384.5, 0);
        slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0,0,0), 0.015, 0, 0.0005, 0.5, 384.5, 0);


        waitForStart();
        stateStartTime = System.currentTimeMillis();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //armControl.moveLiftTo(2400);
//
//            if (!positionSet && lift1.getCurrentPosition() > 250) {
//                positionSet = true;
//                armPositionIdle();
//            }
//
            movement();
//
            armGamepad = gamepad2.getGamepadId() == -1 ? gamepad1 : gamepad2;

            prevArmGamepad.copy(currArmGamepad);
            prevDriveGamepad.copy(currDriveGamepad);

            currArmGamepad.copy(armGamepad);
            currDriveGamepad.copy(driveGamepad);


//            if (armGamepad.left_trigger > 0) {
//                liftUp(armGamepad.left_trigger);
//                lastPos = lift.getCurrentPosition();
//            } else if (armGamepad.right_trigger > 0) {
//                liftDown(armGamepad.right_trigger);
//                lastPos = lift.getCurrentPosition();
//            } else {
//                liftController.setTarget(lastPos);
//                liftController.update();
//            }
//
            switch (robotState) {

                case NEUTRAL:

                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                //reset slide
                                slideController.setTarget(0);
                                time.reset();
                                subState++;

                            case 1:
                                //move lift
                                if (time.seconds() > 1) {
                                    liftController.setTarget(1100-1200);
                                    time.reset();
                                    subState++;

                                }
                                break;
                            case 2:
                                //move elbow
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(0);
                                    elbow2.setPosition(0);
                                    time.reset();
                                    subState ++;
                                }
                                break;
                            case 3:
                                //move wrist
                                if (time.seconds() > 1) {
                                    wrist.setPosition(1);
                                    rotate.setPosition(0);
                                    subState = 0;
                                    // Reset subState for next cycle
                                    subStateDone = true;
                                }
                                break;

                        }
                        break;
                    /*
                    liftController.setTarget(1200);
                    slideController.setTarget(0);
                    //grab.setPosition(0);
                    elbow1.setPosition(0); // sgrab
                    elbow2.setPosition(0);
                    wrist.setPosition(1);
                    rotate.setPosition(0);
                    break;
                    */
                    }



                case FLOOR_GRAB:

                    liftController.setTarget(0-1200);
                    slideController.setTarget(0);
                    //grab.setPosition(0);
                    //rotate.setPosition(0);

                    if (grabbing) {
                        elbow1.setPosition(0.5);
                        elbow2.setPosition(0.5);
                        wrist.setPosition(1);
                    } else {
                        elbow1.setPosition(0.575); // hover
                        elbow2.setPosition(0.575);
                        wrist.setPosition(1);
                    }
                    if(!subStateDone){
                        time.reset();
                        subState = 0;
                        subStateDone = true;
                    }

                    break;

                case SPECIMEN_GRAB:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                //reset slide
                                slideController.setTarget(0);
                                liftController.setTarget(1150-1200);
                                time.reset();
                                subState++;
                                break;


                            case 1:
                                //move lift
                                if (time.seconds() > 1) {
                                    slideController.setTarget(-80);
                                    time.reset();
                                    subState++;
                                    break;
                                }
                                break;
                            case 2:
                                //move elbow
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(.99);
                                    elbow2.setPosition(.99);
                                    time.reset();
                                    subState ++;
                                    break;
                                }
                                break;
                            case 3:
                                //move wrist
                                if (time.seconds() > 1) {
                                    wrist.setPosition(0.575);
                                    rotate.setPosition(0);
                                    subState = 0; // Reset subState for next cycle
                                    subStateDone = true;
                                }
                                break;

                        }
                    /*
                    liftController.setTarget(750);
                    slideController.setTarget(0);
                    elbow1.setPosition(0); // sgrab
                    elbow2.setPosition(0);
                    wrist.setPosition(1);
                    rotate.setPosition(0);
                    */
                        break;
                    }

                case SPECIMEN_TRANSITION:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                slideController.setTarget(0);
                                liftController.setTarget(1100-1200);
                                time.reset();
                                subState++;
                                break;


                            case 1:
                                if (time.seconds() > 1) {
                                    wrist.setPosition(1);


                                    time.reset();
                                    subState ++;
                                }
                                break;
                            case 2:
                                if (time.seconds() > 1) {

                                    rotate.setPosition(0.65);
                                    time.reset();
                                    subState++;

                                }
                            case 3:
                                if(time.seconds() > 1){
                                    elbow1.setPosition(0.6);
                                    elbow2.setPosition(0.6);
                                    subState = 0; // Reset subState for next cycle
                                    subStateDone = true;

                                }

                                break;

                        }
                        break;
                    }



                case SPECIMEN_DROP:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                slideController.setTarget(-85);
                                liftController.setTarget(1150-1200);
                                time.reset();
                                subState++;
                                break;


                            case 1:
                                if(time.seconds() > 1){
                                    wrist.setPosition(0.4);
                                    time.reset();
                                    subState++;
                                    break;
                                }
                            case 2:
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(0.2);
                                    elbow2.setPosition(0.2);

                                    time.reset();
                                    subState = 0;
                                    subStateDone = true;
                                }
                                break;


                        }

                    /*
                    liftController.setTarget(1300);
                    slideController.setTarget(0);
                    //grab.setPosition(0);
                    elbow1.setPosition(1); // sgrab
                    elbow2.setPosition(1);
                    wrist.setPosition(1);
                    rotate.setPosition(0.65);
                    */

                        break;
                    }


                case HIGH_BASKET:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                liftController.setTarget(1150-1200);
                                time.reset();
                                subState++;
                                break;


                            case 1:
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(0.5); // sgrab
                                    elbow2.setPosition(0.5);
                                    time.reset();
                                    subState ++;
                                }
                                break;
                            case 2:
                                if (time.seconds() > 1) {
                                    wrist.setPosition(0);
                                    rotate.setPosition(0.35);
                                    subState = 0; // Reset subState for next cycle
                                    subStateDone = true;
                                }
                                break;

                        }
                    /*
                    liftController.setTarget(1300);
                    slideController.setTarget(2000);
                    //grab.setPosition(0.75);
                    elbow1.setPosition(0.7); // sgrab
                    elbow2.setPosition(0.7);
                    wrist.setPosition(1);
                    rotate.setPosition(0.35);
                    */

                        break;
                    }


                case LOW_BASKET:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                liftController.setTarget(1300-1200);
                                time.reset();
                                subState++;
                                break;

                            case 1:
                                if (time.seconds() > 1) {
                                    slideController.setTarget(1000);
                                    time.reset();
                                    subState++;

                                }
                                break;
                            case 2:
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(0.7); // sgrab
                                    elbow2.setPosition(0.7);
                                    time.reset();
                                    subState ++;
                                }
                                break;
                            case 3:
                                if (time.seconds() > 1) {
                                    wrist.setPosition(1);
                                    rotate.setPosition(0.35);
                                    subState = 0; // Reset subState for next cycle
                                    subStateDone = true;
                                }
                                break;


                        }

                    /*
                    liftController.setTarget(1300);
                    slideController.setTarget(1000);
                    //grab.setPosition(0.75);
                    elbow1.setPosition(0.7); // sgrab
                    elbow2.setPosition(0.7);
                    wrist.setPosition(1);
                    rotate.setPosition(0.35);
                    break;
                    */
                    break;
                    }

        }

            if (armGamepad.dpad_down) {
                robotState = RobotState.FLOOR_GRAB;
                subStateDone = false;
            }

            if (armGamepad.left_bumper) {
                robotState = RobotState.SPECIMEN_DROP;
                subStateDone = false;
            }
            if (armGamepad.dpad_left){
                robotState = RobotState.SPECIMEN_TRANSITION;
                subStateDone = false;
            }
            if (armGamepad.right_bumper) {
                robotState = RobotState.SPECIMEN_GRAB;
                subStateDone = false;
            }

            if (armGamepad.dpad_up) {
                robotState = RobotState.HIGH_BASKET;
                subStateDone = false;
            }


            if (armGamepad.triangle) {
                grab.setPosition(0.4);
            }

            if (armGamepad.cross) {
                grab.setPosition(0.75);
            }

            if (armGamepad.square) {
                rotateClawL();
            }

            if (armGamepad.circle) {

                rotateClawR();
            }
//
            if (armGamepad.left_stick_button) {
                grabbing = true;
            } else if (armGamepad.right_stick_button) {
                grabbing = false;
            }


            if (armGamepad.left_trigger > 0 && slide1.getCurrentPosition() < 3000) {
                double slidePower = armGamepad.left_trigger;
                slide1.setPower(slidePower);
                slide2.setPower(slidePower);
            } else if (armGamepad.right_trigger > 0 && slide1.getCurrentPosition() > 0) {
                double slidePower = armGamepad.right_trigger;
                slide1.setPower(-slidePower);
                slide2.setPower(-slidePower);
            } else {
                slideController.setTarget(slide1.getCurrentPosition());
                slideController.update();
            }

            liftController.update();
        telemetry.addData("State", robotState);
        telemetry.addData("subState", subState);
        telemetry.addData("time", time.seconds());
        telemetry.addData("Is Resetting?", resetting);
        telemetry.addData("Lift Position", lastPos);
        telemetry.addData("Target", target);
        telemetry.addData("rotate index", rotateIndex);
        telemetry.update();
    }
    }
}