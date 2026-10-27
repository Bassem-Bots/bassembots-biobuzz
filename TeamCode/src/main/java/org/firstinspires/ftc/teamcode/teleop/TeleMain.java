package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.DriveTrain;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;

@TeleOp(name="TeleMain", group="Linear Opmode")
public class TeleMain extends LinearOpMode {

    // --- GOBILDA CALIBRATION CONFIGURATION ---
    // STEP 1: Find your Yellow Jacket RPM on gobilda.com and look at the "Encoder Resolution" spec
    // Common examples: 312 RPM = 537.7 | 435 RPM = 383.6 | 223 RPM = 751.8
    private static final double GOBILDA_MOTOR_TICKS = 383.6;

    // STEP 2: Enter your external gear ratio (Driven Gear divided by Driving Gear)
    // If your physical gearing multiplier is exactly 2.4, enter 2.4 here
    private static final double EXTERNAL_GEAR_RATIO = 2.4; // 2.4:1 external gear ratio (48/20)

    // AUTOMATIC CALCULATION: Compiles ticks per single degree of turret rotation
    private static final double TICKS_PER_DEGREE = (GOBILDA_MOTOR_TICKS * EXTERNAL_GEAR_RATIO) / 360.0;

    // STEP 3: Safe Software Boundaries (Maximum ~400 degrees total travel)
    // -200 degrees left to +200 degrees right from your initialized center point
    private static final double MAX_LIMIT_DEG = 180;
    private static final double MIN_LIMIT_DEG = -180;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init(); // Resets your encoder to 0 at your physical starting orientation

        DriveTrain dt = new DriveTrain(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);

        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain control (Gamepad 1)
//            dt.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // --- PROTECTED MANUAL TURRET CONTROL ---
            // Drive the turret manually using Left Stick X
            double turretInputPower = gamepad1.left_stick_x * 0.4; // Scaled to 40% maximum power for precision control

            // Fetch live positions from the hub's built-in encoder readings
            double currentTicks = robot.shooterTurnMotor.getCurrentPosition();
            double currentAngleDeg = currentTicks / TICKS_PER_DEGREE;

            // Soft boundaries block further movement if limits are breached
            if (currentAngleDeg >= MAX_LIMIT_DEG && turretInputPower > 0) {
                turretInputPower = 0; // Cut rightward power
            } else if (currentAngleDeg <= MIN_LIMIT_DEG && turretInputPower < 0) {
                turretInputPower = 0; // Cut leftward power
            }

            // Write safe power calculations directly to hardware
            robot.shooterTurnMotor.setPower(turretInputPower);

            // Real-time telemetry diagnostics for the drive team
            telemetry.addData("Turret Orientation", "%.1f°", currentAngleDeg);
            telemetry.addData("Encoder Counts", currentTicks);
            telemetry.addData("Safe Power Output", "%.2f", turretInputPower);
            telemetry.update();
        }
        dt.stop();
        robot.shooterTurnMotor.setPower(0);
    }
}
