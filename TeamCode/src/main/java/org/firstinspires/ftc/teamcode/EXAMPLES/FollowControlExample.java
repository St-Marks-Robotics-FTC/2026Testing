package org.firstinspires.ftc.teamcode.EXAMPLES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Shows how to start, monitor, and interrupt path following:
 * followPath with/without holdEnd, isBusy, getPose, proximity and timeout
 * guards, holdPoint, breakFollowing, and global vs per-path power scaling.
 */
@Autonomous(name = "EX Follow", group = "EXAMPLES")
public class FollowControlExample extends LinearOpMode {
    private Follower follower;

    private final Pose startPose = new Pose(24, 24, Math.toRadians(0));
    private final Pose midPose = new Pose(72, 24, Math.toRadians(0));
    private final Pose farPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose holdPose = new Pose(24, 72, Math.toRadians(180));

    private PathChain firstLeg;
    private PathChain secondLeg;
    private PathChain slowLeg;

    private Timer pathTimer;
    private int pathState;

    private static final double LEG_TIMEOUT_SECONDS = 5.0;
    private static final double HOLD_SECONDS = 2.0;
    private static final double SLOW_POWER = 0.5;

    public void buildPaths() {
        firstLeg = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        secondLeg = follower.pathBuilder()
                .addPath(new BezierLine(midPose, farPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), farPose.getHeading())
                .build();

        slowLeg = follower.pathBuilder()
                .addPath(new BezierLine(farPose, holdPose))
                .setLinearHeadingInterpolation(farPose.getHeading(), holdPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Plain followPath: releases control when the chain ends.
                follower.followPath(firstLeg);
                setPathState(1);
                break;
            case 1:
                // Wait for the first leg, with a timer fallback in case it stalls.
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > LEG_TIMEOUT_SECONDS) {
                    // holdEnd = true keeps Pedro holding the end pose after arrival.
                    follower.followPath(secondLeg, true);
                    setPathState(2);
                }
                break;
            case 2:
                // Proximity check against the leg's target, plus the same timeout fallback.
                if (follower.getPose().distanceFrom(farPose) < 1
                        || pathTimer.getElapsedTimeSeconds() > LEG_TIMEOUT_SECONDS) {
                    // NOTE: 2.1.2 names this setMaxPowerScaling (plan called it setMaxPowerScalingFactor).
                    follower.setMaxPowerScaling(SLOW_POWER);
                    // NOTE: 2.1.2 has no per-path Path.setScale(); per-path power scaling
                    // uses the followPath(chain, maxPower, holdEnd) overload instead.
                    follower.followPath(slowLeg, SLOW_POWER, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > LEG_TIMEOUT_SECONDS) {
                    follower.setMaxPowerScaling(1);
                    follower.holdPoint(holdPose);
                    setPathState(4);
                }
                break;
            case 4:
                // Hold position briefly, then cancel any active following.
                if (pathTimer.getElapsedTimeSeconds() > HOLD_SECONDS) {
                    follower.breakFollowing();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                // Park in an unused state so no new paths start.
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("path timer", pathTimer.getElapsedTimeSeconds());
            telemetry.update();
        }
    }
}
