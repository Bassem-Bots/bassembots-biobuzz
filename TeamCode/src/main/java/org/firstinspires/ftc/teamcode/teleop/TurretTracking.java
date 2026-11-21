package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.IntakeShooter;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.helpers.Turret;
import org.firstinspires.ftc.teamcode.helpers.TurretConstants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.utils.pedro.Constants;

@TeleOp(name = "Turret Tracking (Localization)", group = "Linear Opmode")
public class TurretTracking extends LinearOpMode {

    // ---------------- HARDWARE ----------------

    private RobotHardware robot;
    private Follower follower;
    private Turret turret;
    private IntakeShooter intakeShooter;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        robot.init();

        follower = Constants.create(hardwareMap);
        follower.setPose(PoseFactory.degrees().of(8, 11, 270));
        follower.update();

        turret = new Turret(robot.shooterTurnMotor);
        intakeShooter = new IntakeShooter(
                robot.shooterMotor,
                robot.intakeMotor,
                robot.transferMotor
        );

        telemetry.addLine("BIOBUZZ Red Turret Initialized");
        telemetry.addLine("Tracking RED HIVE from Pedro localization");
        telemetry.addLine("Tune: dpad up/down = kP, dpad left/right = kD,");
        telemetry.addLine("      bumpers = kS, hold A = fine steps,");
        telemetry.addLine("      B = flip tracking direction");
        telemetry.addLine("Left trigger = intake, Right trigger = shoot (spin up, then feed),");
        telemetry.addLine("X = reverse intake/transfer");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                updateTuning();

                follower.manual(new DrivePowers(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                ));
                follower.update();

                Pose pose = follower.pose();
                boolean scoringHalf = pose.y() > TurretConstants.FIELD_MIDLINE_Y;
                turret.trackToFieldTarget(
                        TurretConstants.targetX("RED"),
                        TurretConstants.targetY(scoringHalf),
                        pose.x(),
                        pose.y(),
                        pose.heading()
                );

                intakeShooter.update(
                        gamepad1.left_trigger > 0.1,
                        gamepad1.right_trigger > 0.1,
                        gamepad1.x
                );

                telemetry.addData("Intake/Shooter", intakeShooter.getState());

                telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Target Half", scoringHalf ? "SCORING" : "AUDIENCE");
                telemetry.addData("Target", "x %.1f  y %.1f",
                        TurretConstants.targetX("RED"),
                        TurretConstants.targetY(scoringHalf));

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
            follower.manual(DrivePowers.zero());
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

}
