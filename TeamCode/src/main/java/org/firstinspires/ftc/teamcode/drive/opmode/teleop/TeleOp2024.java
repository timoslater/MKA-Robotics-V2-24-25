package org.firstinspires.ftc.teamcode.drive.opmode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.utils.AsyncArmActions;


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

    public boolean liftRunning = false;

    private final double minLiftPower = 0.25;

    //public boolean positionSet = false;

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
//        if (!isDropping) {
//            elbow.setPosition(0.296);
//            hand.setPosition(0.92);
//        }

    }

    public void clawClose() {
        claw.setPosition(.64);
//        if (!isDropping) {
//            elbow.setPosition(0.38);
//            hand.setPosition(0.94);
//        };
    }

    public void claw2Open(){
        clawSpecimen.setPosition(.3);
    }
    public void claw2Close(){
        clawSpecimen.setPosition(.64);
    }

    public void rotateClawR() {
        rotate.setPosition(rotate.getPosition()+.01);
    }
    public void rotateClawL() {
        rotate.setPosition(rotate.getPosition()-.01);
    }

//    public void armPositionIdle() {
//        isDropping = false;
//        hand.setPosition(.92);
//        clawOpen();
//    }
//
//    public void armPositionDrop() throws InterruptedException {
//        isDropping = true;
//        elbow.setPosition(0);
//        Thread.sleep(500);
//        hand.setPosition(0);
//    }


     @Override
    public void runOpMode() throws InterruptedException {
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
        //elbow = hardwareMap.get(Servo.class, "elbow");
        //hand = hardwareMap.get(Servo.class, "hand");

        //AsyncArmActions armControl = new AsyncArmActions(0.25, this);



        driveGamepad = gamepad1;


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           //armControl.moveLiftTo(2400);

//            if (!positionSet && lift1.getCurrentPosition() > 250) {
//                positionSet = true;
//                armPositionIdle();
//            }

          movement();

          armGamepad = gamepad2.getGamepadId() == -1 ? gamepad1 : gamepad2;

          if (!liftRunning) {
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
//                  }
                      lastPos = lift1.getCurrentPosition();
                  } else {
                      int positionError = lastPos - lift1.getCurrentPosition();
                      double correctionPower;
                      double kP = 0.25;

                      if (Math.abs(positionError) > 25) {
                          correctionPower = kP * (positionError / 50.0) * (positionError > 0 ? 1 : -1);
                          lift1.setPower(correctionPower);
                          lift2.setPower(correctionPower);
                      } else {
                          lift1.setPower(0);
                          lift2.setPower(0);
                      }
                  }
          }



//          if (armGamepad.dpad_up) {
//              armPositionDrop();
//          }
//
//          if (armGamepad.dpad_down) {
//              armPositionIdle();
//          }

          if (armGamepad.right_bumper && ((lift1.getCurrentPosition() < 600 && slide.getCurrentPosition() < 2800) || lift1.getCurrentPosition() > 600 || resetting)) {
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
          if (armGamepad.dpad_right){
              rotateClawR();
          } else if(armGamepad.dpad_left){
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
          telemetry.addData("Lift Position", lift1.getCurrentPosition());
          telemetry.addData("Slide Position", slide.getCurrentPosition());
          telemetry.update();
        }
    }
}