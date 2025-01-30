package org.firstinspires.ftc.teamcode.drive.opmode.auto.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

//@Autonomous(name = "Rotate While Strafing Test", group = "Auto")
public class RotateWhileStrafingTest extends BaseAuto {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private WaitTimer waitTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    PathChain path1, path2;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.000, 0.000, Point.CARTESIAN),
                                new Point(30.000, 0.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .build();


        path2 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(30.000, 0.000, Point.CARTESIAN),
                                new Point(0.000, 0.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(path1);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    setPathState(0);
                }
                break;


            default:
                if (!follower.isBusy()) {
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
        telemetry.addData("line number", follower.getCurrentPath());
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

