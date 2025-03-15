package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstantsSample;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstantsSpecimen;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

@Autonomous(name = "Sample Auto (Left Side)", group = "Auto")
public class SampleAuto extends BaseAuto {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private WaitTimer waitTimer;
    private MechanismController mechanismController;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    PathChain line1, line2, line3, line4, line5, line6, line7, park;

    private final Pose startPose = new Pose(6.000, 102.250, Math.toRadians(0));

    public void buildPaths() {
        line1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.000, 102.250, Point.CARTESIAN),
                                new Point(18.000, 127.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(325))
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(18.000, 127.000, Point.CARTESIAN),
                                new Point(25.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

        line3 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(25.000, 121.000, Point.CARTESIAN),
                                new Point(18.000, 127.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();

        line4 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(18.000, 127.000, Point.CARTESIAN),
                                new Point(24.500, 129.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                .build();

        line5 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(24.500, 129.250, Point.CARTESIAN),
                                new Point(18.000, 127.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();

        line6 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(18.000, 127.000, Point.CARTESIAN),
                                new Point(41.000, 113.750, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();

        line7 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(41.000, 113.75, Point.CARTESIAN),
                                new Point(18.000, 127.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(315))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(18.000, 127.000, Point.CARTESIAN),
                                new Point(55.000, 120.000, Point.CARTESIAN),
                                new Point(55.000, 89.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);
                mechanismController.moveSlideTo(0);
                mechanismController.moveLiftTo(0);
                mechanismController.highBasketInit();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    mechanismController.moveSlideTo(850);
                    setPathState(2);
                }
                break;

            case 2:
                if (!mechanismController.isBusy()) {
                    mechanismController.highBasketDrop();
                    waitTimer.startTimer(0.5);
                    setPathState(3);
                }
                break;

            case 3:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(4);
                }
                break;

            case 4:
                if (waitTimer.isDone()) {
                    mechanismController.highBasketInit();
                    waitTimer.startTimer(0.25);
                    setPathState(5);
                }
                break;

            case 5:
                if (waitTimer.isDone()) {
                    mechanismController.moveSlideTo(0);
                    setPathState(6);
                }
                break;

            case 6:
                if (!mechanismController.isBusy()) {
                    follower.followPath(line2, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!mechanismController.isBusy() && !follower.isBusy()) {
                    mechanismController.moveLiftTo(-1660);
                    mechanismController.floorGrabHover();
                    waitTimer.startTimer(0.5);
                    setPathState(8);
                }
                break;

            case 8:
                if (!mechanismController.isBusy() && !follower.isBusy() && waitTimer.isDone()) {
                    mechanismController.floorGrabDown();
                    waitTimer.startTimer(0.25);
                    setPathState(9);
                }
                break;

            case 9:
                if (waitTimer.isDone()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(10);
                }
                break;

            case 10:
                if (waitTimer.isDone()) {
                    mechanismController.highBasketInit();
                    mechanismController.moveLiftTo(0);
                    follower.followPath(line3, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy() && !mechanismController.isBusy())  {
                    mechanismController.moveSlideTo(850);
                    setPathState(12);
                }
                break;

            case 12:
                if (!mechanismController.isBusy()) {
                    mechanismController.highBasketDrop();
                    waitTimer.startTimer(0.5);
                    setPathState(13);
                }
                break;

            case 13:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(14);
                }
                break;

            case 14:
                if (waitTimer.isDone()) {
                    mechanismController.highBasketInit();
                    waitTimer.startTimer(0.25);
                    setPathState(15);
                }
                break;

            case 15:
                if (waitTimer.isDone()) {
                    mechanismController.moveSlideTo(0);
                    setPathState(16);
                }
                break;

            case 16:
                if (!mechanismController.isBusy()) {
                    follower.followPath(line4, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!mechanismController.isBusy() && !follower.isBusy()) {
                    mechanismController.moveLiftTo(-1660);
                    mechanismController.floorGrabHover();
                    waitTimer.startTimer(0.5);
                    setPathState(18);
                }
                break;

            case 18:
                if (!mechanismController.isBusy() && !follower.isBusy() && waitTimer.isDone()) {
                    mechanismController.floorGrabDown();
                    waitTimer.startTimer(0.25);
                    setPathState(19);
                }
                break;

            case 19:
                if (waitTimer.isDone()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(20);
                }
                break;

            case 20:
                if (waitTimer.isDone()) {
                    mechanismController.highBasketInit();
                    mechanismController.moveLiftTo(0);
                    follower.followPath(line5, true);
                    setPathState(21);
                }
                break;

            case 21:
                if (!follower.isBusy() && !mechanismController.isBusy())  {
                    mechanismController.moveSlideTo(850);
                    setPathState(22);
                }
                break;

            case 22:
                if (!mechanismController.isBusy()) {
                    mechanismController.highBasketDrop();
                    waitTimer.startTimer(0.5);
                    setPathState(23);
                }
                break;

            case 23:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(24);
                }
                break;

            case 24:
                if (waitTimer.isDone()) {
                    mechanismController.highBasketInit();
                    waitTimer.startTimer(0.25);
                    setPathState(25);
                }
                break;

            case 25:
                if (waitTimer.isDone()) {
                    mechanismController.moveSlideTo(0);
                    setPathState(26);
                }
                break;

            case 26:
                if (!mechanismController.isBusy()) {
//                    mechanismController.moveLiftTo(-1200);
                    //mechanismController.floorGrabHoverHorizontal();
//                    mechanismController.initPose();
                    follower.followPath(line6, true);
                    setPathState(27);
                }
                break;

            case 27:
                if (!follower.isBusy() && !mechanismController.isBusy()) {
                    mechanismController.moveLiftTo(-1660);
                    mechanismController.floorGrabHoverHorizontal();
                    setPathState(28);
                }
                break;

            case 28:
                if (!mechanismController.isBusy() && !follower.isBusy()) {
                    waitTimer.startTimer(0.5);
                    setPathState(29);
                }
                break;

            case 29:
                if (!mechanismController.isBusy() && !follower.isBusy() && waitTimer.isDone()) {
                    mechanismController.floorGrabDownHorizontal();
                    waitTimer.startTimer(0.25);
                    setPathState(30);
                }
                break;

            case 30:
                if (waitTimer.isDone()) {
                    mechanismController.closeClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(31);
                }
                break;

            case 31:
                if (waitTimer.isDone()) {
                    mechanismController.floorGrabHoverHorizontal();
                    mechanismController.moveLiftTo(0);
                    follower.followPath(line7, true);
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy() && !mechanismController.isBusy())  {
                    mechanismController.highBasketInit();
                    mechanismController.moveSlideTo(850);
                    setPathState(33);
                }
                break;

            case 33:
                if (!mechanismController.isBusy()) {
                    mechanismController.highBasketDrop();
                    waitTimer.startTimer(0.5);
                    setPathState(34);
                }
                break;

            case 34:
                if (waitTimer.isDone()) {
                    mechanismController.openClaw();
                    waitTimer.startTimer(0.25);
                    setPathState(35);
                }
                break;

            case 35:
                if (waitTimer.isDone()) {
                    mechanismController.initPose();
                    waitTimer.startTimer(0.5);
                    setPathState(36);
                }
                break;

            case 36:
                if (waitTimer.isDone()) {
                    mechanismController.moveSlideTo(0);
                    setPathState(37);
                }
                break;

            case 37:
                if (!mechanismController.isBusy()) {
                    follower.followPath(park);
                    waitTimer.startTimer(0.5);
                    setPathState(38);
                }
                break;

            case 38:
                if (waitTimer.isDone()) {
                    mechanismController.hangPose();
                    setPathState(39);
                }
                break;

            case 39:
                if (!follower.isBusy()) {
                    mechanismController.hangLockPose();
                    waitTimer.startTimer(0.5);
                    setPathState(40);
                }
                break;

            default:
                if (!follower.isBusy() && !mechanismController.isBusy() && waitTimer.isDone()) {
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
        mechanismController.updateControllers();
        autonomousPathUpdate();

        // Feedback to Driver Hub
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

        mechanismController = new MechanismController();
        mechanismController.initPose();

        Constants.setConstants(FConstantsSample.class, LConstants.class);
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

