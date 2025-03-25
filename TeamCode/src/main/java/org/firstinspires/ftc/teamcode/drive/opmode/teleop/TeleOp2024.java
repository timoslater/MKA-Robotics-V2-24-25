package org.firstinspires.ftc.teamcode.drive.opmode.teleop;



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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.MotorPositionController;
import org.firstinspires.ftc.teamcode.utils.MotorSyncController;
import org.firstinspires.ftc.teamcode.utils.ToggleButton;
//import org.firstinspires.ftc.teamcode.utils.PIDController;
//import org.firstinspires.ftc.teamcode.utils.PIDFController;

import java.util.List;

@Config
@TeleOp(name = "MAIN", group = "Linear Opmode")
public class TeleOp2024 extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftRear = null;
    //private DcMotor revArm = null;
    private DcMotorEx rightRear = null;
    private List<DcMotorEx> motors;
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

    private ToggleButton recoveryToggle = new ToggleButton();
    private ToggleButton speedLimitToggle = new ToggleButton();
    public boolean limited = true;

    public boolean recovering = false;
    private IMU imu;
    private Servo hand;
    private Servo elbow1;
    private Servo elbow2;
    private Servo wrist;
    private int lastPos = 0;

    private boolean grabbing = false;
    private boolean clawClosed = false;

    private int lastSlidePos = 0;
    private boolean isDropping = false;
    private int rotateIndex;
    private double[] rotatePositions = {0.23,0.39,0.52,0.65};

    private MotorPositionController liftController, slideController;
//    private PIDFController angleController;
    public int target = 0;
    private final double ticks = 384.5;

    public boolean liftRunning = false;
    public boolean positionSet = false;
    private double minLiftPower = 0.25;

    private long stateStartTime = 0;
    ElapsedTime time = new ElapsedTime();
    public int subState = 0;
    public int grabState = 0;
    private boolean subStateDone = false;
    private boolean grabDone = false;
    private double headingError;

    private Limelight3A limelight;
    private int[] pipelines = {0,1,2};
    private int pipelineIndex = 0;
    public static double offset = 0.15;
    public static boolean limelightAlign = true;

    private Servo light;

    public enum RobotState {
        FLOOR_GRAB,
        SPECIMEN_GRAB,
        SPECIMEN_TRANSITION,
        SPECIMEN_DROP,
        HIGH_BASKET,
        SPEC_SCORE,
        NEUTRAL,
    }

    public static RobotState robotState = RobotState.FLOOR_GRAB;

    public void movement() {
            double modifier = 1;//nearBoard ? 0.65 : 1;
            double Lpower = 1*modifier;
            double Rpower = limited ? .517 : 1; //0.52*modifier;//*modifier;
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized", "haggis");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(11);


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

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

        light = hardwareMap.get(Servo.class, "light");

//        //AsyncArmActions armControl = new AsyncArmActions(0.25, this);
        Gamepad currDriveGamepad = new Gamepad();
        Gamepad currArmGamepad = new Gamepad();

        Gamepad prevDriveGamepad = new Gamepad();
        Gamepad prevArmGamepad = new Gamepad();

        driveGamepad = gamepad1;
        armGamepad = gamepad1;

        liftController = new MotorPositionController(lift, null,0.004, 0, 0.0004, 0.1, 1425.1, 0);
        slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0,0,0), 0.015, 0, 0.0005, 0.5, 384.5, 0);

        waitForStart();
        stateStartTime = System.currentTimeMillis();
        limelight.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            movement();
//
            armGamepad = gamepad2.getGamepadId() == -1 ? gamepad1 : gamepad2;

            prevArmGamepad.copy(currArmGamepad);
            prevDriveGamepad.copy(currDriveGamepad);

            currArmGamepad.copy(armGamepad);
            currDriveGamepad.copy(driveGamepad);


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
                                    liftController.setTarget(-50);
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
                    if(slide1.getCurrentPosition()<200){
                        liftController.setTarget(-1660);
                        slideController.setTarget(0);
                        //grab.setPosition(0);
                        //rotate.setPosition(0);


                    }
                    if (grabbing) {
                        if(!grabDone){
                            switch(grabState){
                                case 0:
                                    elbow1.setPosition(0.48);
                                    elbow2.setPosition(0.48);
                                    wrist.setPosition(1);
                                    time.reset();
                                    grabState++;
                                    break;
                                case 1:
                                    if(time.seconds()>0.2){
                                        clawClosed = true;
                                        grab.setPosition(0.5);
                                        time.reset();
                                        grabState++;
                                        break;
                                    }
                                case 2:
                                    if(time.seconds()>0.2){
                                        elbow1.setPosition(0.6); // hover
                                        elbow2.setPosition(0.6);
                                        wrist.setPosition(1);
                                        time.reset();
                                        grabState++;
                                        break;
                                    }
                                case 3:
                                    if(time.seconds()>0.2){
                                        slideController.setTarget(25);
                                        time.reset();
                                        grabState = 0;
                                        grabDone = true;
                                        break;
                                    }
                            }
                        }



                    } else {
                        elbow1.setPosition(0.6); // hover
                        elbow2.setPosition(0.6);
                        wrist.setPosition(1);
                        clawClosed = false;
                        grab.setPosition(.95);

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
                                liftController.setTarget(-50);
                                time.reset();
                                subState++;
                                break;




                            case 1:
                                //move wrist
                                if (time.seconds() > .2) {
                                    elbow1.setPosition(.99);
                                    elbow2.setPosition(.99);
                                    wrist.setPosition(0.73);
                                    rotate.setPosition(.23);
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
                                liftController.setTarget(-50);
                                wrist.setPosition(1);
                                time.reset();
                                subState++;
                                break;
                            case 1:
                                if (time.seconds() > .4) {

                                    rotate.setPosition(.8);

                                    elbow1.setPosition(0.3);
                                    elbow2.setPosition(0.3);
                                    wrist.setPosition(0.4);
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

                                elbow1.setPosition(0.12);
                                elbow2.setPosition(0.12);


                                time.reset();
                                subState = 0;
                                subStateDone = true;
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
                    if(!subStateDone && slide1.getCurrentPosition()<200){
                        switch (subState){
                            case 0:
                                liftController.setTarget(-50);
                                elbow1.setPosition(0.5); // sgrab
                                elbow2.setPosition(0.5);

                                wrist.setPosition(0.2);
                                rotate.setPosition(0.52);
                                time.reset();
                                subState = 0; // Reset subState for next cycle
                                subStateDone = true;
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
                /*
                case SPEC_SCORE:
                    if(!subStateDone){
                        switch (subState){
                            case 0:
                                liftController.setTarget(600-1200);
                                time.reset();
                                subState++;
                                break;

                            case 1:
                                if (time.seconds() > 1) {
                                    slideController.setTarget(150);
                                    time.reset();
                                    subState++;

                                }
                                break;
                            case 2:
                                if (time.seconds() > 1) {
                                    elbow1.setPosition(0.4); // sgrab
                                    elbow2.setPosition(0.);
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


                    liftController.setTarget(1300);
                    slideController.setTarget(1000);
                    //grab.setPosition(0.75);
                    elbow1.setPosition(0.7); // sgrab
                    elbow2.setPosition(0.7);
                    wrist.setPosition(1);
                    rotate.setPosition(0.35);
                    break;

                    break;
                    }

                 */

        }
            if(robotState == RobotState.FLOOR_GRAB&&!grabbing){
                if(limelightAlign){
                    if (result != null && !clawClosed) {

                        double[] outputs = result.getPythonOutput();

                        telemetry.addData("Angle", outputs[5]);
                        rotate.setPosition((outputs[5]/255)+offset);
                        telemetry.addData("Valid", result.isValid());
                    }
                }
                else{
                    if (currArmGamepad.circle && !prevArmGamepad.circle){

                        rotateClawR();
                    } else if(currArmGamepad.square && !prevArmGamepad.square){
                        rotateClawL();
                    }
                }
            }
            if(currArmGamepad.touchpad&&!prevArmGamepad.touchpad){
                limelightAlign = !limelightAlign;
            }

            if (armGamepad.dpad_down) {
                robotState = RobotState.FLOOR_GRAB;
                grabbing = false;
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
                clawClosed = false;
                grab.setPosition(.95);
            }

            if (armGamepad.cross) {
                clawClosed = true;
                grab.setPosition(0.5);
            }

            /*
            if (armGamepad.square) {
                offset+=0.01;
            }

            if (armGamepad.circle) {

                offset-=0.01;
            }
            */
            if (currArmGamepad.dpad_right&&!prevArmGamepad.dpad_right) {
                pipelineIndex++;
                limelight.pipelineSwitch(pipelines[pipelineIndex%3]);
            }

            if (armGamepad.left_stick_button) {
                grabbing = true;
                grabDone = false;
            } else if (armGamepad.right_stick_button) {
                grabbing = false;
            }



            if(robotState == RobotState.FLOOR_GRAB){
                if (armGamepad.left_trigger > 0 && (slide1.getCurrentPosition() < 350 || recovering)) {
                    double slidePower = armGamepad.left_trigger;
                    slide1.setPower(slidePower);
                    slide2.setPower(slidePower);
                } else if (armGamepad.right_trigger > 0 && (slide1.getCurrentPosition() > 0 || recovering)) {
                    double slidePower = armGamepad.right_trigger;
                    slide1.setPower(-slidePower);
                    slide2.setPower(-slidePower);
                } else {
                    slideController.setTarget(slide1.getCurrentPosition());
                    slideController.update();
                }
            }
            else{
                if (armGamepad.left_trigger > 0 && (slide1.getCurrentPosition() < 2300 || recovering)) {
                    double slidePower = armGamepad.left_trigger;
                    slide1.setPower(slidePower);
                    slide2.setPower(slidePower);
                } else if (armGamepad.right_trigger > 0 && (slide1.getCurrentPosition() > 0 || recovering)) {
                    double slidePower = armGamepad.right_trigger;
                    slide1.setPower(-slidePower);
                    slide2.setPower(-slidePower);
                } else {
                    slideController.setTarget(slide1.getCurrentPosition());
                    slideController.update();
                }
            }
            /*
            if(robotState != RobotState.FLOOR_GRAB){
                light.setPosition(Math.random()*0.443 + 0.279);
            }
            */
            if (recovering) {
                if (driveGamepad.left_trigger > 0) {
                    lift.setPower(driveGamepad.left_trigger);
                } else if (driveGamepad.right_trigger > 0) {
                    lift.setPower(-driveGamepad.right_trigger);
                } else {
                    lift.setPower(0);
                }
            } else {
                liftController.update();
            }

            if (recoveryToggle.getState(new Boolean[] {driveGamepad.left_bumper, driveGamepad.right_bumper, driveGamepad.left_stick_button, driveGamepad.right_stick_button})) {
                recovering = !recovering;

                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (speedLimitToggle.getState(new Boolean[] {driveGamepad.touchpad})) {
                limited = !limited;
            }

            LLStatus status = limelight.getStatus();



            if (recovering) {
                light.setPosition(0.5);
                telemetry.addData("COLOR", "GREEN (Recovering)");
            }
            else if(!limelightAlign){
                light.setPosition(1);
            }
            else if(status.getPipelineIndex() == 0) {
                light.setPosition(0.28);
                telemetry.addData("COLOR", "RED");
            }
            else if(status.getPipelineIndex()==1) {
                light.setPosition(0.388);
                telemetry.addData("COLOR", "YELLOW");
            }
            else if(status.getPipelineIndex()==2){
                light.setPosition(0.611);
                telemetry.addData("COLOR","BLUE");
            }


            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            telemetry.addData("State", robotState);
            telemetry.addData("subState", subState);
            telemetry.addData("time", time.seconds());
            telemetry.addData("Is Resetting?", resetting);
            telemetry.addData("Lift Position", lastPos);
            telemetry.addData("Target", target);
            telemetry.addData("rotate index", rotateIndex);
            telemetry.addData("headingError", headingError);
            telemetry.addData("recovering", recovering);
            telemetry.addData("limelight align", limelightAlign);
            telemetry.update();
        }
    }

}