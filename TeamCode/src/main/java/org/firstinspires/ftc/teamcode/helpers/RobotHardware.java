package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    private final HardwareMap hwMap;
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    public DcMotor shooterMotor;
    public DcMotorEx shooterTurnMotor;
    public DcMotor transferMotor;
    public DcMotor intakeMotor;

    public CRServo left_intake;
    public CRServo right_intake;

    public RobotHardware(HardwareMap hardwareMap) {
        hwMap = hardwareMap;
    }
    public void init() {
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back");

        shooterMotor = hwMap.get(DcMotor.class, "shooter_motor");
        shooterTurnMotor = hwMap.get(DcMotorEx.class, "shooter_turn_motor");
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");
        transferMotor = hwMap.get(DcMotor.class, "transfer_motor");

        left_intake = hwMap.get(CRServo.class, "left_intake");
        right_intake = hwMap.get(CRServo.class, "right_intake");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTurnMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTurnMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
