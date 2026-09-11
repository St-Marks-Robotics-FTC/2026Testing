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

@Autonomous(name = "EX Full Auto", group = "EXAMPLES")
public class FullAutoExample extends LinearOpMode {

    private Follower follower;
    private final Pose startPose = new Pose(72, 5, Math.toRadians(90));
    private final Pose secondPose = new Pose(72, 96, Math.toRadians(90));
    private final Pose thirdPose = new Pose(12, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(120, 60, Math.toRadians(0));

    private PathChain firstPath, secondPath, thirdPath, endPath;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    public void buildPaths() {
        firstPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, secondPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("Auto", "Start position"))
                .build();

        secondPath = follower.pathBuilder()
                .addPath(new BezierLine(secondPose, thirdPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), thirdPose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("Auto", "Second Position"))
                .build();

        thirdPath = follower.pathBuilder()
                .addPath(new BezierLine(thirdPose, endPose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), endPose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("Auto", "Third position"))
                .build();

        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(thirdPose, new Pose(60, 54), endPose))
                .setLinearHeadingInterpolation(thirdPose.getHeading(), endPose.getHeading())
                .addTemporalCallback(0.5, () -> telemetry.addData("Auto", "End Position"))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstPath);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(secondPath, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(thirdPath, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(endPath, true);
                    setPathState(4);
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
