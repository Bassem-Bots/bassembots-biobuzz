package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.IntakeShooter;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.helpers.Turret;
import org.firstinspires.ftc.teamcode.helpers.TurretConstants;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import android.util.Size;

import java.util.List;

@TeleOp(name = "Turret Tracking", group = "Linear Opmode")
public class TurretTracking extends LinearOpMode {

    // ---------------- HARDWARE ----------------

    private RobotHardware robot;
    private Turret turret;
    private IntakeShooter intakeShooter;

    // ---------------- APRILTAG VISION ----------------

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        robot.init();

        turret = new Turret(robot.shooterTurnMotor);
        intakeShooter = new IntakeShooter(
                robot.shooterMotor,
                robot.intakeMotor,
                robot.transferMotor
        );

        // goBILDA Global Shutter USB Camera (3122-0004-0001)
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        TurretConstants.CAMERA_FX,
                        TurretConstants.CAMERA_FY,
                        TurretConstants.CAMERA_CX,
                        TurretConstants.CAMERA_CY
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(WebcamName.class, TurretConstants.CAMERA_NAME)
                )
                .setCameraResolution(
                        new Size(TurretConstants.CAMERA_WIDTH, TurretConstants.CAMERA_HEIGHT)
                )
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("BIOBUZZ Red Turret Initialized");
        telemetry.addLine("Tracking RED AprilTag Clusters");
        telemetry.addLine("Tune: dpad up/down = kP, dpad left/right = kD,");
        telemetry.addLine("      bumpers = kS, hold A = fine steps,");
        telemetry.addLine("      B = flip tracking direction");
        telemetry.addLine("Left trigger = intake, Right trigger = shoot (spin up, then feed),");
        telemetry.addLine("X = reverse intake/transfer");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                List<AprilTagDetection> detections =
                        aprilTag.getDetections();

                AprilTagClusterDetection target =
                        findRedTarget(detections);

                updateTuning();

                if (target != null) {
                    turret.track(target.ftcPose.bearing);
                } else {
                    turret.stop();
                }

                intakeShooter.update(
                        gamepad1.left_trigger > 0.1,
                        gamepad1.right_trigger > 0.1,
                        gamepad1.x
                );

                telemetry.addData("Intake/Shooter", intakeShooter.getState());

                telemetry.addData(
                        "Target Found",
                        target != null
                );

                if (target != null) {

                    telemetry.addData(
                            "Cluster",
                            target.metadata.name
                    );

                    telemetry.addData(
                            "Bearing",
                            "%.2f",
                            target.ftcPose.bearing
                    );

                    telemetry.addData(
                            "Roll",
                            "%.2f",
                            target.ftcPose.roll
                    );

                    telemetry.addData(
                            "Range",
                            "%.2f",
                            target.ftcPose.range
                    );

                    telemetry.addData(
                            "Cluster Visible",
                            "%d%%",
                            target.percentClusterFound
                    );
                }

                telemetry.addData(
                        "Turret Angle",
                        "%.2f",
                        turret.getDegrees()
                );

                telemetry.addData(
                        "Motor Power",
                        "%.4f",
                        turret.getPower()
                );

                telemetry.addData("kP", "%.4f", TurretConstants.kP);
                telemetry.addData("kD", "%.5f", TurretConstants.kD);
                telemetry.addData("kS", "%.3f", TurretConstants.kS);
                telemetry.addData("Track Sign (B flips)", "%.0f", TurretConstants.TRACK_SIGN);

                double deg = turret.getDegrees();
                if (deg >= TurretConstants.MAX_LIMIT_DEG || deg <= TurretConstants.MIN_LIMIT_DEG) {
                    telemetry.addLine("AT SOFT LIMIT - if it ran away from the target, press B to flip direction");
                }

                telemetry.update();
            }
        } finally {
            turret.stop();
            intakeShooter.stop();
            visionPortal.close();
        }
    }

    // ---------------- LIVE TUNING ----------------
    // Edge-detected so each press changes a gain once. Values reset to
    // TurretConstants defaults on restart, so copy final numbers back into that file.

    private boolean lastUp, lastDown, lastLeft, lastRight, lastLb, lastRb, lastB;

    private void updateTuning() {

        double scale = gamepad1.a ? 0.1 : 1.0;

        if (gamepad1.dpad_up && !lastUp) {
            TurretConstants.kP += 0.001 * scale;
        }
        if (gamepad1.dpad_down && !lastDown) {
            TurretConstants.kP = Math.max(0, TurretConstants.kP - 0.001 * scale);
        }
        if (gamepad1.dpad_right && !lastRight) {
            TurretConstants.kD += 0.0005 * scale;
        }
        if (gamepad1.dpad_left && !lastLeft) {
            TurretConstants.kD = Math.max(0, TurretConstants.kD - 0.0005 * scale);
        }
        if (gamepad1.right_bumper && !lastRb) {
            TurretConstants.kS += 0.01 * scale;
        }
        if (gamepad1.left_bumper && !lastLb) {
            TurretConstants.kS = Math.max(0, TurretConstants.kS - 0.01 * scale);
        }

        if (gamepad1.b && !lastB) {
            TurretConstants.TRACK_SIGN = -TurretConstants.TRACK_SIGN;
        }

        lastB = gamepad1.b;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastLb = gamepad1.left_bumper;
        lastRb = gamepad1.right_bumper;
    }

    // ---------------- RED CLUSTER DETECTION ----------------

    private AprilTagClusterDetection findRedTarget(
            List<AprilTagDetection> detections) {

        AprilTagClusterDetection bestTarget = null;

        double bestScore = Double.NEGATIVE_INFINITY;

        for (AprilTagDetection detection : detections) {

            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata == null ||
                    cluster.ftcPose == null) {
                continue;
            }

            String name = cluster.metadata.name;

            // Only track RED alliance clusters
            if (name == null ||
                    !name.toUpperCase().startsWith("RED")) {
                continue;
            }

            // Ignore upside-down (unscorable) CELLS
            if (Math.abs(cluster.ftcPose.roll) >= 90) {
                continue;
            }

            // Prefer a cluster with more visible tags
            double score = cluster.percentClusterFound;

            if (score > bestScore) {
                bestScore = score;
                bestTarget = cluster;
            }
        }

        return bestTarget;
    }
}
