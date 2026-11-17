package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    public DriveTrain(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
    }

    /**
     * @param forward - positive drives forward (-left stick y)
     * @param strafe  - positive strafes right (left stick x)
     * @param turn    - positive turns clockwise (right stick x)
     */
    public void drive(double forward, double strafe, double turn) {
        double leftFront = forward + strafe + turn;
        double rightFront = forward - strafe - turn;
        double leftBack = forward - strafe + turn;
        double rightBack = forward + strafe - turn;

        double[] wheels = {leftFront, rightFront, leftBack, rightBack};
        double max = 0;

        for (double v : wheels) max = Math.max(max, Math.abs(v));
        if (max > 1.0) {
            for (int i = 0; i < wheels.length; i++) wheels[i] /= max;
        }

        leftFrontMotor.setPower(wheels[0]);
        rightFrontMotor.setPower(wheels[1]);
        leftBackMotor.setPower(wheels[2]);
        rightBackMotor.setPower(wheels[3]);
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
