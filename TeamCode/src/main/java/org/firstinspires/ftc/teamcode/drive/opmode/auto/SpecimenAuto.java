package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

@Autonomous(name = "Specimen Auto (Right Side)", group = "Auto")
public class SpecimenAuto extends BaseAuto {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private WaitTimer waitTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(11.800, 61.700, Math.toRadians(90));

    PathChain prePath, line1, line2To6, line7, line8To9, line10, line11To12, line13, line14To15, line16, line17To18;

    PathChain submersiblePointTest, wallPointTest;


    private MainArm mainArm;
    private SideArm sideArm;

    public void buildPaths() {
        prePath = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(11.800, 61.700, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();


        line1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(41.200, 58.200, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        line2To6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(41.200, 58.200, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(30.000, 38.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(30.000, 38.000, Point.CARTESIAN),
                                new Point(70.000, 35.000, Point.CARTESIAN),
                                new Point(70.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(70.000, 25.000, Point.CARTESIAN),
                                new Point(26.000, 17.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(26.000, 17.000, Point.CARTESIAN),
                                new Point(42.000, 30.000, Point.CARTESIAN),
                                new Point(18.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(18.500, 30.000, Point.CARTESIAN),
                                new Point(11.800, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        line8To9 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(11.800, 30.000, Point.CARTESIAN),
                                new Point(18.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(18.500, 30.000, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(41.200, 66.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        line11To12 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(41.200, 66.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(18.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .build();

        line13 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(18.500, 30.000, Point.CARTESIAN),
                                new Point(11.800, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        line14To15 = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(11.800, 30.000, Point.CARTESIAN),
                                new Point(18.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(18.500, 30.000, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                .build();

        line16 = follower.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(41.200, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        line17To18 = follower.pathBuilder()
                .addPath(
                        // Line 17
                        new BezierLine(
                                new Point(41.200, 70.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 18
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(12.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                sideArm.moveLiftTo(2400);
                follower.followPath(prePath,true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && !sideArm.isBusy()) {
                    follower.followPath(line1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    sideArm.moveLiftTo(1525);
                    setPathState(3);
                }
                break;

            case 3:
                if (!sideArm.isBusy()) {
                    sideArm.openClaw();
                    sideArm.moveLiftTo(0);
                    setPathState(4);
                }
                break;


            case 4:
                if (!sideArm.isBusy()) {
                    follower.followPath(line2To6, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    waitTimer.startTimer(1);
                    setPathState(6);
                }
                break;

            case 6:
                if (waitTimer.isDone()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(line7, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    sideArm.closeClaw();
                    setPathState(8);
                }
                break;

            case 8:
                if (!sideArm.isBusy()) {
                    follower.setMaxPower(1.0);
                    sideArm.moveLiftTo(2400);
                    follower.followPath(line8To9);
                    setPathState(9);
                }
                break;

            case 9:
                if (!sideArm.isBusy() && !follower.isBusy()) {
                    follower.followPath(line10, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    sideArm.moveLiftTo(1525);
                    setPathState(11);
                }
                break;

            case 11:
                if (!sideArm.isBusy()) {
                    sideArm.openClaw();
                    sideArm.moveLiftTo(0);
                    setPathState(12);
                }
                break;

            case 12:
                if (!sideArm.isBusy()) {
                    follower.followPath(line11To12, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    waitTimer.startTimer(1);
                    setPathState(14);
                }
                break;

            case 14:
                if (waitTimer.isDone()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(line13, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    sideArm.closeClaw();
                    setPathState(16);
                }
                break;

            case 16:
                if (!sideArm.isBusy()) {
                    follower.setMaxPower(1.0);
                    sideArm.moveLiftTo(2400);
                    follower.followPath(line14To15);
                    setPathState(17);
                }

            case 17:
                if (!sideArm.isBusy() && !sideArm.isBusy()) {
                    follower.followPath(line16, true);
                    setPathState(18);
                }
                break;


            case 18:
                if (!follower.isBusy()) {
                    sideArm.moveLiftTo(1525);
                    setPathState(19);
                }
                break;

            case 19:
                if (!sideArm.isBusy()) {
                    sideArm.openClaw();
                    sideArm.moveLiftTo(0);
                    setPathState(20);
                }
                break;

            case 20:
                if (!sideArm.isBusy()) {
                    follower.followPath(line17To18);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy() && !sideArm.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        mainArm.update();
        sideArm.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("line number", follower.getCurrentPath());
        telemetry.addData("servoTimer delta", sideArm.getServoTimeDelta());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitTimer = new WaitTimer();



        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        mainArm = new MainArm(hardwareMap);
        sideArm = new SideArm(hardwareMap);

        sideArm.closeClaw();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

