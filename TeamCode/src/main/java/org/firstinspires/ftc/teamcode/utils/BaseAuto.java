package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseAuto extends OpMode {

    public class WaitTimer {
        private ElapsedTime timer;
        private double endTime;

        public WaitTimer() {
            timer = new ElapsedTime();
        }

        public void startTimer(double waitTime) {
            endTime = timer.seconds() + waitTime;
        }

        public boolean isDone() {
            return timer.seconds() > endTime;
        }

    }

    public enum MechanismState {
        PICKUP,
        GRAB,
        SPECIMEN_UP,
        SPECIMEN_DOWN,
        HIGH_BASKET_UP,
        HIGH_BASKET_DOWN,
        MANUAL
    }

    public class MechanismController {

        private DcMotorEx lift1, lift2, slide1, slide2;
        private Servo elbow1, elbow2, wrist, rotate, grab;
        private MotorPositionController liftController, slideController;
        private boolean isBusy = false;


        public MechanismController() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//            lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
//            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
//            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//            slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
//            slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
//            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slide2.setDirection(DcMotorSimple.Direction.REVERSE);

            elbow1 = hardwareMap.get(ServoImplEx.class, "elbow1");
            elbow1.setDirection(Servo.Direction.REVERSE);

            elbow2 = hardwareMap.get(ServoImplEx.class, "elbow2");

            wrist = hardwareMap.get(ServoImplEx.class, "wrist");

            rotate = hardwareMap.get(ServoImplEx.class, "rotate");

            grab = hardwareMap.get(ServoImplEx.class, "grab");

//            liftController = new MotorPositionController(lift1, lift2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);
//            slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);
        }


        public void openClaw() {
            grab.setPosition(0.4);
        }

        public void closeClaw() {
            grab.setPosition(0.73);
        }

        public void specimenGrabPosition() {
            elbow1.setPosition(.99);
            elbow2.setPosition(.99);
            wrist.setPosition(0.575);
            rotate.setPosition(0);
            openClaw();
        }

        public void endPosition() {
            specimenGrabPosition();
            wrist.setPosition(0.1);
        }

        public void liftSpecimenAway() {
            wrist.setPosition(1);
        }

        public void preSpecimenDrop() {
            rotate.setPosition(0.65);
            elbow1.setPosition(0.6);
            elbow2.setPosition(0.6);
            wrist.setPosition(0.4);
        }

        public void specimentScore() {
            elbow1.setPosition(0.2);
            elbow2.setPosition(0.2);
        }

        public void initPose() {
            elbow1.setPosition(0.1);
            elbow2.setPosition(0.1);
            wrist.setPosition(0.4);
            rotate.setPosition(0.65);
            closeClaw();
        }

//        public void moveLiftTo(int targetPos) {
//            liftController.setTarget(targetPos);
//        }
//
//        public void moveSlideTo(int targetPos) {
//            slideController.setTarget(targetPos);
//        }

        public boolean isBusy() {
            return isBusy;
        }

//        public void update() {
//            liftController.update();
//            slideController.update();
//        }
    }
}