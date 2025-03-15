package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstantsSpecimen;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

//@Autonomous(name = "Specimen Auto (Right Side)", group = "Auto")
public class SpecimenAuto extends BaseAuto {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private WaitTimer waitTimer;

    private MechanismController mechanismController;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final int TOTAL_CYCLES = 4;
    private int cyclesCompleted = 0;

    private final Pose startPose = new Pose(6.5, 54.25, Math.toRadians(0));

    PathChain line1, line2To8, line9, line10, line11, line12, specimenSlide, park;

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.500, 54.250, Point.CARTESIAN),
                                new Point(38.500, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line2To8 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(38.500, 70.000, Point.CARTESIAN),
                                new Point(8.000, 40.000, Point.CARTESIAN),
                                new Point(48.000, 34.000, Point.CARTESIAN),
                                new Point(63.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(63.000, 23.000, Point.CARTESIAN),
                                new Point(31.250, 23.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(31.250, 23.500, Point.CARTESIAN),
                                new Point(80.000, 23.500, Point.CARTESIAN),
                                new Point(53.000, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(53.000, 13.500, Point.CARTESIAN),
                                new Point(31.250, 13.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(31.250, 13.500, Point.CARTESIAN),
                                new Point(80.000, 13.500, Point.CARTESIAN),
                                new Point(53.000, 7.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(53.000, 7.500, Point.CARTESIAN),
                                new Point(31.250, 7.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(31.250, 7.500, Point.CARTESIAN),
                                new Point(16.000, 33.450, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line9 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(16.000, 33.450, Point.CARTESIAN),
                                new Point(12.750, 33.450, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line10 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(12.750, 33.450, Point.CARTESIAN),
                                new Point(16.000, 33.450, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line11 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(16.000, 33.450, Point.CARTESIAN),
                                new Point(30.000, 75.000, Point.CARTESIAN),
                                new Point(38.500, 75.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line12 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(38.500, 70.000, Point.CARTESIAN),
                                new Point(30.000, 67.000, Point.CARTESIAN),
                                new Point(16.000, 33.450, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        specimenSlide = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(38.500, 75.000, Point.CARTESIAN),
                                new Point(38.500, 70.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(38.250, 67.000, Point.CARTESIAN),
                                new Point(18.500, 70.000, Point.CARTESIAN),
                                new Point(11.750, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                mechanismController.preSpecimenDrop();
                follower.followPath(line1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    mechanismController.specimentScore();
                    waitTimer.startTimer(0.25);
                    setPathState(2);
                }
                break;

//            case 100:
//                if (waitTimer.isDone() && !follower.isBusy()) {
//                    follower.followPath(specimenSlide);
//                    setPathState(2);
//                }
//                break;

            case 2:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(3);
                }
                break;


            case 3:
                if (!follower.isBusy() && waitTimer.isDone()) {
                    follower.followPath(line2To8, true);
                    waitTimer.startTimer(3);
                    setPathState(4);
                }
                break;

            case 300:
                if (waitTimer.isDone()) {
                    mechanismController.specimenGrabPosition();
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    waitTimer.startTimer(1);
                    setPathState(5);
                }
                break;

            case 5:
                if (waitTimer.isDone() && !follower.isBusy()) {
                    follower.followPath(line9, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(0.5);
                    setPathState(9);
                }
                break;

//            case 7:
//                if (waitTimer.isDone()) {
//                    mechanismController.liftSpecimenAway();
//                    mechanismController.preSpecimenDrop();
//                    waitTimer.startTimer(2);
//                    setPathState(9);
//                }
//                break;

//            case 8:
//                if (waitTimer.isDone()) {
//                    follower.followPath(line10, true);
//                    setPathState(9);
//                }
//                break;

//            case 8:
//                if (!follower.isBusy()) {
//                    mechanismController.preSpecimenDrop();
//                    setPathState(9);
//                }
//                break;


            case 9:
                if (!follower.isBusy() && waitTimer.isDone()) {
                    mechanismController.liftSpecimenAway();
                    follower.followPath(line11, true);
                    waitTimer.startTimer(1);
                    setPathState(10);
                }
                break;

            case 10:
                if (waitTimer.isDone()) {
                    mechanismController.preSpecimenDrop();
                    setPathState(200);
                }
                break;

            case 200:
                if (waitTimer.isDone() && !follower.isBusy()) {
                    mechanismController.specimentScore();
                    follower.followPath(specimenSlide);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    cyclesCompleted++;
                    setPathState(12);
                }
                break;

            case 12:
                if (waitTimer.isDone()) {
                    if (cyclesCompleted < TOTAL_CYCLES) {
                        follower.followPath(line12, true);
                        waitTimer.startTimer(0.25);
                        buildPaths(); // rebuild all paths
                        setPathState(4);
                    } else {
                        follower.followPath(park);
                        mechanismController.endPosition();
                        waitTimer.startTimer(0.5);
                        setPathState(-1);
                    }
                }
                break;

            default:
                if (!follower.isBusy() && waitTimer.isDone()) {
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

