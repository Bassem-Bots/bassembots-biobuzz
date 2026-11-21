package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.DriveTrain;
import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.helpers.Turret;

@TeleOp(name="TeleMain", group="Linear Opmode")
public class TeleMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init(); // Resets your encoder to 0 at your physical starting orientation

        DriveTrain dt = new DriveTrain(robot.leftFrontDrive, robot.rightFrontDrive, robot.leftBackDrive, robot.rightBackDrive);
        Turret turret = new Turret(robot.shooterTurnMotor);

        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain control (Gamepad 1)
            dt.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Protected manual turret control (Left Stick X, soft limits enforced)
            turret.manual(gamepad1.left_stick_x);

            telemetry.addData("Turret Angle", "%.1f°", turret.getDegrees());
            telemetry.addData("Encoder Counts", turret.getTicks());
            telemetry.addData("Safe Power Output", "%.2f", turret.getPower());
            telemetry.update();
        }
        dt.stop();
        turret.stop();
    }
}
