package org.firstinspires.ftc.teamcode.drive.opmode.auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.utils.BaseAuto;

@Config
@Autonomous(name = "Far Auto (Left)", group = "Autonomous")
public class FarAuto extends BaseAuto {
    //private Servo elbow;
    private DcMotor lift1, lift2;
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(35.8, 61.7, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        SideArm sideArm = new SideArm(hardwareMap);
        MainArm mainArm = new MainArm(hardwareMap);
        //elbow = hardwareMap.get(Servo.class, "elbow");

        lift1 = hardwareMap.get(DcMotor.class, "lift");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectoryActionBuilder toRung1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(12, 36))
                .turnTo(0);

        Action toRung1Action = toRung1.build();

        TrajectoryActionBuilder toRung2 = toRung1.fresh()
                .strafeTo(new Vector2d(12, 30))
                .turnTo(0);

        Action toRung2Action = toRung2.build();

        TrajectoryActionBuilder afterRung1 = toRung2.fresh()
                .strafeTo(new Vector2d(12, 36))
                .strafeTo(new Vector2d(40, 36))//6200
                .turnTo(Math.PI)
                .strafeTo(new Vector2d(40, 10))
                .strafeTo(new Vector2d(28, 10));

        Action afterRung1Action = afterRung1.build();

        TrajectoryActionBuilder afterRung2 = afterRung1.fresh()
                .strafeTo(new Vector2d(28, 10));

        Action afterRung2Action = afterRung2.build();

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(6);

        Action waitAction = wait.build();

        Actions.runBlocking(new SequentialAction(
                sideArm.closeClaw(),
                sideArm.resetEncoder(),
                mainArm.resetEncoder()
        ));

        while (!isStopRequested() && !opModeIsActive()) {
            // in between initialization and match start
            //elbow.setPosition(); // from testing
//            int positionError = 300 - lift1.getCurrentPosition();
//            double correctionPower;
//            double kP = 0.25;
//
//            if (Math.abs(positionError) > 25) {
//                correctionPower = kP * (positionError / 50.0) * (positionError > 0 ? 1 : -1);
//                lift1.setPower(correctionPower);
//                lift2.setPower(correctionPower);
//            } else {
//                lift1.setPower(0);
//                lift2.setPower(0);
//            }
        }

        waitForStart();

        if (isStopRequested()) return;

        // start autonomous path

//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                sideArm.moveTo(1000, 0.6),
//                                toRungAction
//                        ),
//                        sideArm.moveTo(800, 0.6),
//                        sideArm.openClaw(),
//                        afterRungAction
//                )
//        );
        //1400

        int positionError = 300 - lift1.getCurrentPosition();
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

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                sideArm.moveTo(2400, 1),
                                toRung1Action
                        ),
                        //waitAction,
                        toRung2Action,
                        sideArm.moveTo(1400, 1),
                        sideArm.openClaw(),
                        //waitAction,
                        new ParallelAction(
                                afterRung1Action,
                                sideArm.moveTo(0, 1),
                                mainArm.moveTo(3075, 0.5)
                        )

                        //afterRung2Action
                )
        );

        /* For Example:

        Actions.runBlocking(
                new SequentialAction(
                        path1,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        park
                )
        );

        */
    }
}
