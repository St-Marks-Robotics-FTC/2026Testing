package org.firstinspires.ftc.teamcode.EXAMPLES;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Teaches Pedro's geometry building blocks: Pose construction, heading units,
 * and the BezierLine / BezierCurve / BezierPoint curve types.
 *
 * <p>No driving happens here on purpose: nothing calls followPath(). The
 * PathChains below only show how each curve slots into follower.pathBuilder().
 */
@TeleOp(name = "EX Pose Geometry", group = "EXAMPLES") //tis an example
public class PosesAndGeometryExample extends OpMode {

    private Follower follower; //Tis a follower

    // Poses: Pedro measures x/y in inches and heading in radians (CCW positive).
    private final Pose startPose = new Pose(72, 5, Math.toRadians(270));
    private final Pose secondPose = new Pose(72, 96, Math.toRadians(180));
    private final Pose thirdPose = new Pose(12, 60);
    private final Pose endPose = new Pose(120, 60, Math.toRadians(90));

    // Raw geometry objects, built directly (not generated).
    private BezierLine line;
    private BezierCurve quadraticCurve; // 3 points: start, control, end
    private BezierCurve cubicCurve; // 4 points: start, two controls, end
    private BezierPoint stopPoint; // single-pose stop point

    // The same curves wrapped as drivable chains via the builder.
    private PathChain lineChain;
    private PathChain quadraticChain;
    private PathChain cubicChain;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Straight segment between two poses.
        line = new BezierLine(startPose, endPose);

        // Quadratic curve: start, one control point, end.
        quadraticCurve = new BezierCurve(startPose, controlPose, endPose);

        // Cubic curve: start, two control points, end. Pose implements
        // FuturePose, so plain Poses fit the BezierCurve(FuturePose...) varargs.
        cubicCurve = new BezierCurve(startPose, controlPose, extraControlPose, endPose);

        // A BezierPoint pins a single pose; followers treat it as a stop point.
        stopPoint = new BezierPoint(stopPose);

        // Each curve drops into follower.pathBuilder() unchanged.
        lineChain = follower.pathBuilder()
                .addPath(line)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        quadraticChain = follower.pathBuilder()
                .addPath(quadraticCurve)
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        cubicChain = follower.pathBuilder()
                .addPath(cubicCurve)
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void loop() {
        follower.update();

        // Pose values: read back exactly what was constructed above.
        telemetry.addData("start", "x=%.1f y=%.1f heading=%.1f deg",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("mid", "x=%.1f y=%.1f heading=%.1f deg",
                midPose.getX(), midPose.getY(), Math.toDegrees(midPose.getHeading()));
        telemetry.addData("end", "x=%.1f y=%.1f heading=%.1f deg",
                endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading()));

        // BezierLine endpoints.
        telemetry.addData("line start", "x=%.1f y=%.1f",
                line.getFirstControlPoint().getX(), line.getFirstControlPoint().getY());
        telemetry.addData("line end", "x=%.1f y=%.1f",
                line.getLastControlPoint().getX(), line.getLastControlPoint().getY());

        // Quadratic (3-point) BezierCurve: start, control, end.
        telemetry.addData("quad start", "x=%.1f y=%.1f",
                quadraticCurve.getFirstControlPoint().getX(), quadraticCurve.getFirstControlPoint().getY());
        telemetry.addData("quad control", "x=%.1f y=%.1f",
                quadraticCurve.getSecondControlPoint().getX(), quadraticCurve.getSecondControlPoint().getY());
        telemetry.addData("quad end", "x=%.1f y=%.1f",
                quadraticCurve.getLastControlPoint().getX(), quadraticCurve.getLastControlPoint().getY());

        // Cubic (4-point) BezierCurve: start, two controls, end.
        telemetry.addData("cubic start", "x=%.1f y=%.1f",
                cubicCurve.getFirstControlPoint().getX(), cubicCurve.getFirstControlPoint().getY());
        telemetry.addData("cubic control 1", "x=%.1f y=%.1f",
                cubicCurve.getSecondControlPoint().getX(), cubicCurve.getSecondControlPoint().getY());
        telemetry.addData("cubic end", "x=%.1f y=%.1f",
                cubicCurve.getLastControlPoint().getX(), cubicCurve.getLastControlPoint().getY());

        // BezierPoint stop point.
        telemetry.addData("stop", "x=%.1f y=%.1f heading=%.1f deg",
                stopPoint.getFirstControlPoint().getX(),
                stopPoint.getFirstControlPoint().getY(),
                Math.toDegrees(stopPoint.getFirstControlPoint().getHeading()));

        // Live localizer pose for comparison against the constructed poses.
        telemetry.addData("robot", "x=%.1f y=%.1f heading=%.1f deg",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
