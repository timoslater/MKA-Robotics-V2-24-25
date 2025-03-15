package org.firstinspires.ftc.teamcode.drive.opmode.auto.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstantsSample {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14;

        FollowerConstants.xMovement = 81.55;
        FollowerConstants.yMovement = 66.68;

        FollowerConstants.forwardZeroPowerAcceleration = -38;
        FollowerConstants.lateralZeroPowerAcceleration = -72;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.3,0,0.03,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.075,0,0.02,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.03,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1,0,0.02,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0,0.000005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.005,0,0.00000025,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 1000;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.05;
        FollowerConstants.pathEndTranslationalConstraint = 0.001;
        FollowerConstants.pathEndHeadingConstraint = 0.00175;
    }
}
