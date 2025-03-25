package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstantsSpecimen;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

@Config
@Autonomous(name = "Specimen Auto V2 (Right Side)", group = "Auto")
public class SpecimenAutoV2 extends BaseAuto {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private WaitTimer waitTimer;

    private MechanismController mechanismController;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */

    public static double GRAB_WAIT = 0.1, LIFT_AWAY_WAIT = 0.05, SCORE_WAIT = 0.135, HUMAN_WAIT = 0; //0.3;
    private int pathState;

    private final Pose startPose = new Pose(6.5, 54.25, Math.toRadians(0));

    PathChain line1, line2To6, line7, line7Extended, line8, line9, line10, line11, line12, line13, line14, line15, line16, line17, line18, line19, line20, line21;

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.500, 54.250, Point.CARTESIAN),
                                new Point(40.500, 74.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line2To6 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(40.500, 74.000, Point.CARTESIAN),
                                new Point(8.000, 40.000, Point.CARTESIAN),
                                new Point(60.000, 38.000, Point.CARTESIAN),
                                new Point(57.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(57.000, 23.000, Point.CARTESIAN),
                                new Point(31.250, 23.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(31.250, 23.500, Point.CARTESIAN),
                                new Point(61.000, 23.500, Point.CARTESIAN),
                                new Point(49.000, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.7)
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(49.000, 13.500, Point.CARTESIAN),
                                new Point(31.250, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(31.250, 13.500, Point.CARTESIAN),
                                new Point(75.000, 6.250, Point.CARTESIAN),
                                new Point(30.000, 5.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.5)
//                .addPath(
//                        // Line 6
//                        new BezierCurve(
//                                new Point(31.250, 13.500, Point.CARTESIAN),
//                                new Point(85.000, 6.500, Point.CARTESIAN),
//                                new Point(20.000, 7.500, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(30.000, 5.000, Point.CARTESIAN),
                                new Point(12.750, 5.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTValueConstraint(0.995)
                .build();


        line8 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(12.750, 7.500, Point.CARTESIAN),
                                new Point(18.000, 72.000, Point.CARTESIAN),
                                new Point(40.500, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line9 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(40.500, 72.000, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(30.000, 33.450, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(18.000, 38.250, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line11 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(12.750, 38.250, Point.CARTESIAN),
                                new Point(30.000, 38.250, Point.CARTESIAN),
                                new Point(30.000, 70.000, Point.CARTESIAN),
                                new Point(40.500, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line12 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(40.500, 70.000, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(30.000, 33.450, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line13 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(18.000, 38.250, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line14 = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(12.750, 38.250, Point.CARTESIAN),
                                new Point(30.000, 38.250, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(40.500, 68.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line15 = follower.pathBuilder()
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(40.500, 68.000, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(30.000, 33.450, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line16 = follower.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(18.000, 38.250, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line17 = follower.pathBuilder()
                .addPath(
                        // Line 17
                        new BezierCurve(
                                new Point(12.750, 38.250, Point.CARTESIAN),
                                new Point(30.000, 38.250, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(40.500, 66.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

//        line18 = follower.pathBuilder()
//                .addPath(
//                        // Line 18
//                        new BezierCurve(
//                                new Point(40.500, 66.000, Point.CARTESIAN),
//                                new Point(18.500, 70.000, Point.CARTESIAN),
//                                new Point(11.750, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();

        line18 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(40.500, 66.000, Point.CARTESIAN),
                                new Point(30.000, 71.000, Point.CARTESIAN),
                                new Point(30.000, 38.250, Point.CARTESIAN),
                                new Point(12.750, 38.250, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line19 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(16.000, 33.450, Point.CARTESIAN),
                                new Point(12.750, 33.450, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line20 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(12.750, 33.450, Point.CARTESIAN),
                                new Point(30.000, 33.450, Point.CARTESIAN),
                                new Point(18.000, 127.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();

        line21 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(18.000, 127.000, Point.CARTESIAN),
                                new Point(23.000, 122.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mechanismController.preSpecimenDrop();
                mechanismController.moveSlideTo(50);
                follower.followPath(line1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(SCORE_WAIT);
                    setPathState(2);
                }
                break;

            case 2:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(3);
                }
                break;
// IMPORTANT

            case 3:
                if (!follower.isBusy() && waitTimer.isDone()) {
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line2To6, true);
                    waitTimer.startTimer(1.5);
                    setPathState(4);
                }
                break;

            case 4:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    setPathState(5);
                }
                break;


            case 5:
                if (!follower.isBusy()) {
//                    waitTimer.startTimer(HUMAN_WAIT);
                    waitTimer.startTimer(0);
                    setPathState(8);
                }
                break;

//            case 6:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line7, true);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    waitTimer.startTimer(0.5);
//                    setPathState(8);
//                }
//                break;

            case 8:
                if (waitTimer.isDone() && !follower.isBusy()) {
                    follower.followPath(line7, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(10);
                }
                break;


            case 10:
                if (waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    waitTimer.startTimer(LIFT_AWAY_WAIT);
                    setPathState(11);
                }
                break;

            case 11:
                if (waitTimer.isDone()) {
                    follower.followPath(line8, true);
                    mechanismController.moveSlideTo(50);
                    mechanismController.preSpecimenDrop();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(SCORE_WAIT);
                    setPathState(13);
                }
                break;

            case 13:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(14);
                }
                break;

            case 14:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line9, true);
                    setPathState(17);
                }
                break;

//            case 15:
//                if (!follower.isBusy()) {
//                    waitTimer.startTimer(HUMAN_WAIT);
//                    setPathState(16);
//                }
//                break;
//
//            case 16:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line10, true);
//                    setPathState(17);
//                }
//                break;

            case 17:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(18);
                }
                break;

            case 18:
                if (waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    waitTimer.startTimer(LIFT_AWAY_WAIT);
                    setPathState(19);
                }
                break;

            case 19:
                if (waitTimer.isDone()) {
                    follower.followPath(line11, true);
                    mechanismController.moveSlideTo(50);
                    mechanismController.preSpecimenDrop();
                    setPathState(20);
                }
                break;

            case 20:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(SCORE_WAIT);
                    setPathState(21);
                }
                break;

            case 21:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(22);
                }
                break;

            case 22:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line12, true);
                    setPathState(25);
                }
                break;

//            case 23:
//                if (!follower.isBusy()) {
//                    waitTimer.startTimer(HUMAN_WAIT);
//                    setPathState(24);
//                }
//                break;
//
//            case 24:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line13, true);
//                    setPathState(25);
//                }
//                break;

            //

            case 25:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(26);
                }
                break;

            case 26:
                if (waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    waitTimer.startTimer(LIFT_AWAY_WAIT);
                    setPathState(27);
                }
                break;

            case 27:
                if (waitTimer.isDone()) {
                    follower.followPath(line14, true);
                    mechanismController.moveSlideTo(50);
                    mechanismController.preSpecimenDrop();
                    setPathState(28);
                }
                break;

            case 28:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(SCORE_WAIT);
                    setPathState(29);
                }
                break;

            case 29:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(30);
                }
                break;


            case 30:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line15, true);
                    setPathState(33);
                }
                break;

//            case 31:
//                if (!follower.isBusy()) {
//                    waitTimer.startTimer(HUMAN_WAIT);
//                    setPathState(32);
//                }
//                break;
//
//            case 32:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line16, true);
//                    setPathState(33);
//                }
//                break;

            case 33:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(34);
                }
                break;

            case 34:
                if (waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    waitTimer.startTimer(LIFT_AWAY_WAIT);
                    setPathState(35);
                }
                break;

            case 35:
                if (waitTimer.isDone()) {
                    follower.followPath(line17, true);
                    mechanismController.moveSlideTo(50);
                    mechanismController.preSpecimenDrop();
                    setPathState(36);
                }
                break;

            case 36:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(SCORE_WAIT);
                    setPathState(37);
                }
                break;

            case 37:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(38);
                }
                break;

            case 38:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line18, true);
                    setPathState(41);
                }
                break;

//            case 39:
//                if (!follower.isBusy()) {
//                    waitTimer.startTimer(HUMAN_WAIT);
//                    setPathState(40);
//                }
//                break;
//
//            case 40:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line19, true);
//                    setPathState(41);
//                }
//                break;

            case 41:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(GRAB_WAIT);
                    setPathState(42);
                }
                break;

            case 42:
                if (waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    waitTimer.startTimer(LIFT_AWAY_WAIT);
                    setPathState(43);
                }
                break;
            case 43:
                if (!follower.isBusy()) {
                    mechanismController.highBasketInit();
                    follower.followPath(line20, true);
                    waitTimer.startTimer(1.5);
                    setPathState(44);
                }
                break;

            case 44:
                if (waitTimer.isDone()) {
                    mechanismController.moveSlideTo(850);
                    setPathState(45);
                }
                break;

            case 45:
                if (!mechanismController.isBusy()) {
                    mechanismController.highBasketDrop();
                    waitTimer.startTimer(0.5);
                    setPathState(46);
                }
                break;

            case 46:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.1);
                    setPathState(47);
                }
                break;

            case 47:
                if (waitTimer.isDone()) {
                    mechanismController.initPose();
                    mechanismController.moveSlideTo(0);
                    follower.followPath(line21, true);
                    waitTimer.startTimer(0.5);
                    setPathState(48);
                }
                break;

            case 48:
                if (waitTimer.isDone()) {
                    mechanismController.endPosition();
                    waitTimer.startTimer(1);
                    setPathState(49);
                }
                break;

            default:
                if (!follower.isBusy() && waitTimer.isDone() && !mechanismController.isBusy()) {
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
        autonomousPathUpdate();
        mechanismController.updateControllers();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        waitTimer = new WaitTimer();

        mechanismController = new MechanismController();
        mechanismController.initPose();

        Constants.setConstants(FConstantsSpecimen.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
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

