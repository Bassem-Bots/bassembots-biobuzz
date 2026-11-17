package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.DriveTrain;
import org.firstinspires.ftc.teamcode.helpers.IntakeShooter;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.helpers.Turret;
import org.firstinspires.ftc.teamcode.helpers.TurretConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Shared TeleOp for both alliances. The turret tracks the AprilTag cluster whose
 * name starts with the alliance name, and falls back to manual control on demand.
 *
 * Gamepad 1:
 *   Left stick          drive / strafe
 *   Right stick X       turn
 *   Left trigger        intake
 *   Right trigger       shoot (spin up, then feed)
 *   X                   reverse intake / transfer
 *   Dpad up / down      shooter power +/- 5%
 *   Y                   toggle turret TRACKING / MANUAL
 *   Bumpers (manual)    turret left / right (gamepad 2 left stick X also works)
 *
 * Gamepad 2 (live tracking tuning, resets on robot restart, copy results into
 * TurretConstants):
 *   Dpad up / down      kP +/-
 *   Dpad left / right   kD -/+
 *   Bumpers             kS +/-
 *   B                   flip tracking direction
 *   Hold A              fine steps
 */
public abstract class AllianceTeleOp extends LinearOpMode {

    private static final double SHOOTER_POWER_STEP = 0.05;
    private static final double MIN_SHOOTER_POWER = 0.1;

    private enum TurretMode { TRACKING, MANUAL }

    private final String alliance;

    private RobotHardware robot;
    private DriveTrain driveTrain;
    private Turret turret;
    private IntakeShooter intakeShooter;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private TurretMode turretMode = TurretMode.TRACKING;
    private boolean lastY, lastUp, lastDown;
    private boolean lastUp2, lastDown2, lastLeft2, lastRight2, lastLb2, lastRb2, lastB2;

    protected AllianceTeleOp(String alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        robot.init();

        driveTrain = new DriveTrain(
                robot.leftFrontDrive,
                robot.rightFrontDrive,
                robot.leftBackDrive,
                robot.rightBackDrive
        );
        turret = new Turret(robot.shooterTurnMotor);
        intakeShooter = new IntakeShooter(
                robot.shooterMotor,
                robot.intakeMotor,
                robot.transferMotor
        );

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        TurretConstants.CAMERA_FX,
                        TurretConstants.CAMERA_FY,
                        TurretConstants.CAMERA_CX,
                        TurretConstants.CAMERA_CY
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, TurretConstants.CAMERA_NAME))
                .setCameraResolution(
                        new Size(TurretConstants.CAMERA_WIDTH, TurretConstants.CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("BIOBUZZ " + alliance + " TeleOp initialized");
        telemetry.addLine("Turret starts in TRACKING (Y toggles MANUAL)");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                updateButtons();

                driveTrain.drive(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                );

                AprilTagClusterDetection target =
                        findTarget(aprilTag.getDetections());

                if (turretMode == TurretMode.MANUAL) {
                    double input = Range.clip(
                            (gamepad1.right_bumper ? 1 : 0)
                                    - (gamepad1.left_bumper ? 1 : 0)
                                    + gamepad2.left_stick_x,
                            -1, 1);
                    turret.manual(input);
                } else if (target != null) {
                    turret.track(target.ftcPose.bearing);
                } else {
                    turret.stop();
                }

                intakeShooter.update(
                        gamepad1.left_trigger > 0.1,
                        gamepad1.right_trigger > 0.1,
                        gamepad1.x
                );
                updateIntakeServos();

                addTelemetry(target);
            }
        } finally {
            driveTrain.stop();
            turret.stop();
            intakeShooter.stop();
            robot.left_intake.setPower(0);
            robot.right_intake.setPower(0);
            visionPortal.close();
        }
    }

    private void updateButtons() {
        if (gamepad1.y && !lastY) {
            turretMode = turretMode == TurretMode.TRACKING
                    ? TurretMode.MANUAL
                    : TurretMode.TRACKING;
        }
        if (gamepad1.dpad_up && !lastUp) {
            intakeShooter.setShooterPower(
                    intakeShooter.getShooterPower() + SHOOTER_POWER_STEP);
        }
        if (gamepad1.dpad_down && !lastDown) {
            intakeShooter.setShooterPower(Math.max(
                    MIN_SHOOTER_POWER,
                    intakeShooter.getShooterPower() - SHOOTER_POWER_STEP));
        }

        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        updateTuning();
    }

    private void updateTuning() {
        double scale = gamepad2.a ? 0.1 : 1.0;

        if (gamepad2.dpad_up && !lastUp2) {
            TurretConstants.kP += 0.001 * scale;
        }
        if (gamepad2.dpad_down && !lastDown2) {
            TurretConstants.kP = Math.max(0, TurretConstants.kP - 0.001 * scale);
        }
        if (gamepad2.dpad_right && !lastRight2) {
            TurretConstants.kD += 0.0005 * scale;
        }
        if (gamepad2.dpad_left && !lastLeft2) {
            TurretConstants.kD = Math.max(0, TurretConstants.kD - 0.0005 * scale);
        }
        if (gamepad2.right_bumper && !lastRb2) {
            TurretConstants.kS += 0.01 * scale;
        }
        if (gamepad2.left_bumper && !lastLb2) {
            TurretConstants.kS = Math.max(0, TurretConstants.kS - 0.01 * scale);
        }
        if (gamepad2.b && !lastB2) {
            TurretConstants.TRACK_SIGN = -TurretConstants.TRACK_SIGN;
        }

        lastUp2 = gamepad2.dpad_up;
        lastDown2 = gamepad2.dpad_down;
        lastLeft2 = gamepad2.dpad_left;
        lastRight2 = gamepad2.dpad_right;
        lastLb2 = gamepad2.left_bumper;
        lastRb2 = gamepad2.right_bumper;
        lastB2 = gamepad2.b;
    }

    // The intake servos run alongside the intake motor, same as in autonomous.
    private void updateIntakeServos() {
        double power;
        switch (intakeShooter.getState()) {
            case INTAKING:
            case FEEDING:
                power = IntakeShooter.INTAKE_POWER;
                break;
            case REVERSING:
                power = -IntakeShooter.INTAKE_POWER;
                break;
            default:
                power = 0;
        }
        robot.left_intake.setPower(power);
        robot.right_intake.setPower(power);
    }

    private void addTelemetry(AprilTagClusterDetection target) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret Mode (Y)", turretMode);
        telemetry.addData("Shooter Power (dpad up/down)", "%.0f%%",
                intakeShooter.getShooterPower() * 100);
        telemetry.addData("Intake/Shooter", intakeShooter.getState());
        telemetry.addData("Target Found", target != null);

        if (target != null) {
            telemetry.addData("Cluster", target.metadata.name);
            telemetry.addData("Bearing", "%.2f", target.ftcPose.bearing);
            telemetry.addData("Range", "%.2f", target.ftcPose.range);
        }

        telemetry.addData("Turret Angle", "%.2f", turret.getDegrees());
        telemetry.addData("Turret Power", "%.4f", turret.getPower());
        telemetry.addData("kP / kD / kS", "%.4f / %.5f / %.3f",
                TurretConstants.kP, TurretConstants.kD, TurretConstants.kS);
        telemetry.addData("Track Sign (gp2 B flips)", "%.0f", TurretConstants.TRACK_SIGN);
        telemetry.update();
    }

    private AprilTagClusterDetection findTarget(List<AprilTagDetection> detections) {

        AprilTagClusterDetection bestTarget = null;
        double bestScore = Double.NEGATIVE_INFINITY;

        for (AprilTagDetection detection : detections) {

            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;

            if (cluster.metadata == null || cluster.ftcPose == null) {
                continue;
            }

            String name = cluster.metadata.name;

            if (name == null || !name.toUpperCase().startsWith(alliance)) {
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
