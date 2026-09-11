package org.firstinspires.ftc.teamcode.EXAMPLES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Full autonomous routine built entirely by hand with {@code follower.pathBuilder()}.
 *
 * <p>Clones the poses and 8-state flow of {@code ExampleAuto}: preload, three
 * pickup/score cycles, then parking off the starting line. Scoring and intake
 * actions are {@code telemetry.addData} placeholders inside temporal callbacks,
 * so this example needs no robot hardware beyond the follower localizer.
 */
@Autonomous(name = "EX Full Auto", group = "EXAMPLES")
public class FullAutoExample extends LinearOpMode {

    private Follower follower;
    private final Pose startPose = new Pose(22, 122, Math.toRadians(324));
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180));
    private final Pose endPose = new Pose(60, 105);

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "scoring preload"))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "intaking pickup 1"))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "scoring pickup 1"))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 54), pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "intaking pickup 2"))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, new Pose(60, 54), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "scoring pickup 2"))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, new Pose(60, 30), pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "intaking pickup 3"))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, new Pose(60, 30), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("EX Full Auto", "scoring pickup 3"))
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(leave, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
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
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        opmodeTimer.resetTimer();
        setPathState(0);

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
