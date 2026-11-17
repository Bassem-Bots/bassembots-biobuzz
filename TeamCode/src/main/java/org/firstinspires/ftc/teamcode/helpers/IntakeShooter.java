package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeShooter {

    // Flip to -1 if the flywheel spins the wrong way
    public static double SHOOTER_POWER = 0.85;
    // Flip signs below if the intake / transfer pulls balls the wrong way
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    // No encoder on the flywheel, so wait this long for it to reach speed before feeding
    public static double SPINUP_SECONDS = 0.8;

    public enum State { IDLE, INTAKING, SPINNING_UP, FEEDING, REVERSING }

    private final DcMotor shooter;
    private final DcMotor intake;
    private final DcMotor transfer;
    private final ElapsedTime spinTimer = new ElapsedTime();

    private State state = State.IDLE;
    private double shooterPower = SHOOTER_POWER;

    public IntakeShooter(DcMotor shooter, DcMotor intake, DcMotor transfer) {
        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
    }

    /**
     * Call every loop. Shooting wins over reversing, which wins over intaking.
     *
     * @param intaking  run the intake and transfer inward
     * @param shooting  spin the flywheel, then feed the ball through the transfer
     * @param reversing run the intake and transfer backwards to clear a jam
     */
    public void update(boolean intaking, boolean shooting, boolean reversing) {
        State previous = state;

        if (shooting) {
            if (previous != State.SPINNING_UP && previous != State.FEEDING) {
                spinTimer.reset();
            }
            state = spinTimer.seconds() >= SPINUP_SECONDS ? State.FEEDING : State.SPINNING_UP;
        } else if (reversing) {
            state = State.REVERSING;
        } else if (intaking) {
            state = State.INTAKING;
        } else {
            state = State.IDLE;
        }

        switch (state) {
            case SPINNING_UP:
                set(shooterPower, 0, 0);
                break;
            case FEEDING:
                set(shooterPower, INTAKE_POWER, TRANSFER_POWER);
                break;
            case INTAKING:
                set(0, INTAKE_POWER, TRANSFER_POWER);
                break;
            case REVERSING:
                set(0, -INTAKE_POWER, -TRANSFER_POWER);
                break;
            default:
                set(0, 0, 0);
        }
    }

    public void stop() {
        state = State.IDLE;
        set(0, 0, 0);
    }

    private void set(double shooterPower, double intakePower, double transferPower) {
        shooter.setPower(shooterPower);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public void setShooterPower(double power) {
        shooterPower = Math.max(0, Math.min(1, power));
    }

    public State getState() {
        return state;
    }
}
