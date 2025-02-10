package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
        private AxonServo elbow1, elbow2, wrist, rotate, grab;
        private MotorPositionController liftController, slideController;
        private boolean isBusy = false;


        public MechanismController() {
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

            elbow1 = new AxonServo(hardwareMap.get(ServoImplEx.class, "elbow1"), hardwareMap.get(AnalogInput.class, "elbow1Input"));

            elbow2 = new AxonServo(hardwareMap.get(ServoImplEx.class, "elbow2"), hardwareMap.get(AnalogInput.class, "elbow2Input"));
            elbow2.setDirection(Servo.Direction.REVERSE);

            wrist = new AxonServo(hardwareMap.get(ServoImplEx.class, "wrist"), hardwareMap.get(AnalogInput.class, "wristInput"));

            rotate = new AxonServo(hardwareMap.get(ServoImplEx.class, "rotate"), hardwareMap.get(AnalogInput.class, "rotateInput"));

            grab = new AxonServo(hardwareMap.get(ServoImplEx.class, "grab"), hardwareMap.get(AnalogInput.class, "grabInput"));

            liftController = new MotorPositionController(lift1, lift2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);
            slideController = new MotorPositionController(slide1, slide2, new MotorSyncController(0, 0, 0), 0, 0, 0, 0, 0, 0);
        }


        public void openClaw() {
            grab.setPosition(0);
        }

        public void closeClaw() {
            grab.setPosition(0);
        }

        public void moveLiftTo(int targetPos) {
            liftController.setTarget(targetPos);
        }

        public void moveSlideTo(int targetPos) {
            slideController.setTarget(targetPos);
        }

        public boolean isBusy() {
            return isBusy;
        }

        public void update() {
            liftController.update();
            slideController.update();
        }
    }
}