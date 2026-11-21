package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.IntakeShooter;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.helpers.Turret;
import org.firstinspires.ftc.teamcode.helpers.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.pedro.Constants;

/**
 * Shared TeleOp for both alliances. The turret tracks the known alliance HIVE
 * position using Pedro localization, and falls back to manual control on demand.
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
    private Follower follower;
    private Turret turret;
    private IntakeShooter intakeShooter;

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

        follower = Constants.create(hardwareMap);
        PoseFactory poseFactory = alliance.equals("BLUE")
                ? PoseFactory.degrees().mirrorY(70.25).mirrorY(70)
                : PoseFactory.degrees();
        follower.setPose(poseFactory.of(8, 11, 270));
        follower.update();

        turret = new Turret(robot.shooterTurnMotor);
        intakeShooter = new IntakeShooter(
                robot.shooterMotor,
                robot.intakeMotor,
                robot.transferMotor
        );

        telemetry.addLine("BIOBUZZ " + alliance + " TeleOp initialized");
        telemetry.addLine("Turret starts in LOCALIZATION TRACKING (Y toggles MANUAL)");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                updateButtons();

                follower.manual(new DrivePowers(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                ));
                follower.update();

                Pose pose = follower.pose();

                if (turretMode == TurretMode.MANUAL) {
                    double input = Range.clip(
                            (gamepad1.right_bumper ? 1 : 0)
                                    - (gamepad1.left_bumper ? 1 : 0)
                                    + gamepad2.left_stick_x,
                            -1, 1);
                    turret.manual(input);
                } else if (pose != null) {
                    boolean scoringHalf = pose.y() > TurretConstants.FIELD_MIDLINE_Y;
                    turret.trackToFieldTarget(
                            TurretConstants.targetX(alliance),
                            TurretConstants.targetY(scoringHalf),
                            pose.x(),
                            pose.y(),
                            pose.heading()
                    );
                }

                intakeShooter.update(
                        gamepad1.left_trigger > 0.1,
                        gamepad1.right_trigger > 0.1,
                        gamepad1.x
                );
                updateIntakeServos();

                addTelemetry(pose);
            }
        } finally {
            follower.manual(DrivePowers.zero());
            turret.stop();
            intakeShooter.stop();
            robot.left_intake.setPower(0);
            robot.right_intake.setPower(0);
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

    private void addTelemetry(Pose pose) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret Mode (Y)", turretMode);
        telemetry.addData("Shooter Power (dpad up/down)", "%.0f%%",
                intakeShooter.getShooterPower() * 100);
        telemetry.addData("Intake/Shooter", intakeShooter.getState());
        if (pose != null) {
            boolean scoringHalf = pose.y() > TurretConstants.FIELD_MIDLINE_Y;
            telemetry.addData(
                    "Pose",
                    "x %.1f  y %.1f  heading %.1f",
                    pose.x(),
                    pose.y(),
                    Math.toDegrees(pose.heading())
            );
            telemetry.addData("Target Half", scoringHalf ? "SCORING" : "AUDIENCE");
            telemetry.addData(
                    "Target",
                    "x %.1f  y %.1f",
                    TurretConstants.targetX(alliance),
                    TurretConstants.targetY(scoringHalf)
            );
        }

        telemetry.addData("Turret Angle", "%.2f", turret.getDegrees());
        telemetry.addData("Turret Power", "%.4f", turret.getPower());
        telemetry.addData("kP / kD / kS", "%.4f / %.5f / %.3f",
                TurretConstants.kP, TurretConstants.kD, TurretConstants.kS);
        telemetry.addData("Track Sign (gp2 B flips)", "%.0f", TurretConstants.TRACK_SIGN);
        telemetry.update();
    }

}
