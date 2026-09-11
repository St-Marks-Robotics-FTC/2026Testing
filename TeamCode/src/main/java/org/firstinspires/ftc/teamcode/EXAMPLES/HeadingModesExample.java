package org.firstinspires.ftc.teamcode.EXAMPLES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drives one straight BezierLine four ways, each in its own PathChain, to show
 * every heading interpolation mode. Press A to advance to the next mode.
 */
@TeleOp(name = "EX Headings", group = "EXAMPLES")
public class HeadingModesExample extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(24, 24, Math.toRadians(0));
    private final Pose endPose = new Pose(72, 24, Math.toRadians(90));

    private PathChain constantChain;
    private PathChain linearChain;
    private PathChain tangentChain;
    private PathChain reverseTangentChain;

    private static final String[] MODE_NAMES = {
            "Constant heading",
            "Linear heading",
            "Tangent heading",
            "Reversed tangent heading"
    };

    private int mode = 0;

    private PathChain chainForMode(int index) {
        switch (index) {
            case 1:
                return linearChain;
            case 2:
                return tangentChain;
            case 3:
                return reverseTangentChain;
            default:
                return constantChain;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        constantChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        linearChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        tangentChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setTangentHeadingInterpolation()
                .build();

        reverseTangentChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setTangentHeadingInterpolation()
                // NOTE: PathBuilder has no reverseHeadingInterpolation() in 2.1.2
                // (that method lives on Path); the builder equivalent is setReversed().
                .setReversed()
                .build();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);
        mode = 0;
        follower.followPath(chainForMode(mode));
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy() && gamepad1.aWasPressed()) {
            mode = (mode + 1) % MODE_NAMES.length;
            follower.followPath(chainForMode(mode));
        }

        telemetry.addData("mode", "%d (%s)", mode, MODE_NAMES[mode]);
        telemetry.addData("press A for next mode", "");
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
