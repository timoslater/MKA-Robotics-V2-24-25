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

import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.BaseAuto;

@Autonomous(name = "Specimen Auto (Right Side)", group = "Auto")
public class SpecimenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(11.800, 61.700, Math.toRadians(90));

    Path line1;
    PathChain line2To3, line4To5, line6To9, line10To11, line12To14;


    private BaseAuto.MainArm mainArm;
    private BaseAuto.SideArm sideArm;

    public void buildPaths() {
        line1 = new Path(
                        // Line 1
                        new BezierLine(
                                new Point(11.800, 61.700, Point.CARTESIAN),
                                new Point(38.000, 61.700, Point.CARTESIAN)
                        )
                );
        line1.setConstantHeadingInterpolation(Math.toRadians(90));

        line2To3 = follower.pathBuilder()
                .addPath(
                     // Line 2
                    new BezierLine(
                            new Point(38.000, 61.700, Point.CARTESIAN),
                            new Point(30.000, 61.700, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(16.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .build();

        line4To5 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(16.000, 35.000, Point.CARTESIAN),
                                new Point(30.000, 30.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(38.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        line6To9 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(38.000, 61.700, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(30.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(30.000, 35.000, Point.CARTESIAN),
                                new Point(70.000, 35.000, Point.CARTESIAN),
                                new Point(70.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(70.000, 25.000, Point.CARTESIAN),
                                new Point(16.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        line10To11 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(16.000, 25.000, Point.CARTESIAN),
                                new Point(30.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(30.000, 25.000, Point.CARTESIAN),
                                new Point(16.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        line12To14 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(16.000, 25.000, Point.CARTESIAN),
                                new Point(30.000, 25.000, Point.CARTESIAN),
                                new Point(30.000, 40.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(30.000, 40.000, Point.CARTESIAN),
                                new Point(30.000, 61.700, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(30.000, 61.700, Point.CARTESIAN),
                                new Point(38.000, 61.700, Point.CARTESIAN)
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
                follower.followPath(line1,true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(line2To3);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line4To5);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line6To9);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line10To11);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(line12To14);
                    setPathState(6);
                }
                break;
            case 6:
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
        mainArm.update();
        sideArm.update();
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

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        mainArm = new BaseAuto.MainArm(hardwareMap);
        sideArm = new BaseAuto.SideArm(hardwareMap);
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

