package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.MotorController;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public abstract class BaseAuto extends OpMode {

    public class WaitTimer {
        private ElapsedTime timer;
        private double startTime;
        private double endTime;
        private boolean isActive;

        public WaitTimer() {
            timer = new ElapsedTime();
        }

        public void startTimer(double waitTime) {
            isActive = true;
            startTime = timer.seconds();
            endTime = startTime + waitTime;
        }

        public boolean isDone() {
            if(timer.seconds() > endTime) {
                isActive = false;
                return true;
            }
            return false;
        }

    }
    public class MainArm {
        private DcMotorEx lift1, lift2;
        private MotorController liftController;
        private DcMotorEx slide;
        private MotorController slideController;

        public MainArm(HardwareMap hardwareMap) {
            lift1 = hardwareMap.get(DcMotorEx.class, "lift");
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);

            liftController = new MotorController(lift1, lift2, 0.009, 0, 0.0001, 0.05, 384.5, 0);

            slide = hardwareMap.get(DcMotorEx.class, "slide");
            slide.setDirection(DcMotor.Direction.REVERSE);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideController = new MotorController(slide, 0.003, 0, 0.00001, 0, 751.8 , 0);


        }

        public void moveLiftTo(int targetPos) {
            liftController.setTarget(targetPos);
            liftController.update();
        }

        public void moveSlideTo(int targetPos) {
            slideController.setTarget(targetPos);
            slideController.update();
        }

        public void update() {
            liftController.update();
            slideController.update();
        }

        public boolean isBusy() {
            return liftController.isBusy() || slideController.isBusy();
        }
    }

    public class SideArm {
        private DcMotorEx lift;
        private MotorController liftController;
        private Servo grabber;

        private ElapsedTime servoTimer;
        private boolean isServoBusy = false;
        private double lastServoStartTime = 0;

        public SideArm(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "specimen");
            lift.setDirection(DcMotor.Direction.REVERSE);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftController = new MotorController(lift, 0.02, 0, 0.0001, 0, 537.7,0);

            grabber = hardwareMap.get(Servo.class, "clawSpecimen");

            servoTimer = new ElapsedTime();
        }

        private void setServoStartTime() {
            lastServoStartTime = servoTimer.seconds();
        }

        public double getServoTimeDelta() {
            return servoTimer.seconds() - lastServoStartTime;
        }

        public void openClaw() {
            setServoStartTime();
            grabber.setPosition(0.3);
        }

        public void closeClaw() {
            setServoStartTime();
            grabber.setPosition(0.66);
        }

        public void moveLiftTo(int targetPos) {
            liftController.setTarget(targetPos);
            liftController.update();
        }

        public boolean isBusy() {
            return liftController.isBusy() || (servoTimer.seconds() - lastServoStartTime < 1);
        }

        public void update() {
            liftController.update();
        }
    }


/*

    public class Claw {
        private Servo grabber;
        private Servo rotator;

        public Claw(HardwareMap hardwareMap) {
            grabber = hardwareMap.get(Servo.class, "grabber");
            rotator = hardwareMap.get(Servo.class, "rotator");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabber.setPosition(0.84);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                grabber.setPosition(0.5);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

        public class RotateClawTo implements Action {

            double targetPosition;

            public RotateClawTo(double targetPosition) {
                this.targetPosition = targetPosition;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(this.targetPosition);
                return false;
            }
        }
        public Action rotateClawTo(double targetPosition) {
            return new RotateClawTo(targetPosition);
        }
    }
    */
}