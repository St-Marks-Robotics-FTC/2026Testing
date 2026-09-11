package org.firstinspires.ftc.teamcode.EXAMPLES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.callbacks.TemporalCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Hand-written multi-path chain: a triangle of three BezierLines with one
 * builder call per commented block. No Ivy-generated code is used here.
 */
@Autonomous(name = "EX Chains", group = "EXAMPLES")
public class PathChainBuildingExample extends LinearOpMode {

    private Follower follower;
    private PathChain triangle;

    private final Pose cornerA = new Pose(24, 24, Math.toRadians(0));
    private final Pose cornerB = new Pose(72, 24, Math.toRadians(90));
    private final Pose cornerC = new Pose(48, 72, Math.toRadians(180));

    private boolean temporalFired;
    private boolean pathFired;
    private boolean parametricFired;

    public void buildPaths() {
        triangle = follower.pathBuilder()
                // Global deceleration applies to every path in this chain.
                .setGlobalDeceleration()

                // Leg 1 of the triangle, with linear heading interpolation.
                .addPath(new BezierLine(cornerA, cornerB))
                .setLinearHeadingInterpolation(cornerA.getHeading(), cornerB.getHeading())

                // Temporal callback: fires 0.5s after the current (first) path starts.
                .addTemporalCallback(0.5, () -> temporalFired = true)

                // Parametric callback: fires when the current path reaches t = 0.5.
                .addParametricCallback(0.5, () -> parametricFired = true)

                // Leg 2 of the triangle.
                .addPath(new BezierLine(cornerB, cornerC))
                .setLinearHeadingInterpolation(cornerB.getHeading(), cornerC.getHeading())

                // Path callback: 2.1.2 has no addPathCallback(n, ...), so target
                // path index 1 explicitly with addCallback(callback, pathIndex).
                .addCallback(new TemporalCallback(1, 0.0, () -> pathFired = true), 1)

                // Leg 3 of the triangle, back to the start.
                .addPath(new BezierLine(cornerC, cornerA))
                .setLinearHeadingInterpolation(cornerC.getHeading(), cornerA.getHeading())

                // Zero-power acceleration: 2.1.2 has no
                // setZeroPowerAccelerationMultiplier on the builder (zero-power
                // accel lives on FollowerConstants instead), so use the closest
                // builder-level braking knob.
                .setBrakingStrength(1.0)

                // Path-end timeout: 2.1.2 has no setPathEndTimeoutConstraint, so
                // constrain the most recently added (final) path instead.
                .setTimeoutConstraint(3.0)

                // Path-end velocity: 2.1.2 has no setPathEndVelocityConstraint.
                .setVelocityConstraint(40.0)

                // Path-end translational: 2.1.2 has no
                // setPathEndTranslationalConstraint.
                .setTranslationalConstraint(0.5)

                // Path-end heading: 2.1.2 has no setPathEndHeadingConstraint.
                .setHeadingConstraint(0.05)

                .build();
    }

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(cornerA);

        waitForStart();
        follower.followPath(triangle);

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("busy", follower.isBusy());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("temporal", temporalFired);
            telemetry.addData("path", pathFired);
            telemetry.addData("parametric", parametricFired);
            telemetry.update();
        }
    }
}
