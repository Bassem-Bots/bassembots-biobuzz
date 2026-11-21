package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    private final DcMotorEx motor;
    private final int startingPosition;
    private final ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private boolean hasLastError = false;
    private double dTerm = 0;
    private double power = 0;

    public Turret(DcMotorEx motor) {
        this.motor = motor;
        this.startingPosition = motor.getCurrentPosition();
        timer.reset();
    }

    /**
     * Manual control with soft limits.
     *
     * @param input - joystick value from -1 to 1
     */
    public void manual(double input) {
        applyPower(input * TurretConstants.MANUAL_POWER_SCALE);
        resetPd();
    }

    /**
     * PD tracking toward a target.
     *
     * @param bearingDeg - target bearing relative to the turret in degrees (0 = centered)
     */
    public void track(double bearingDeg) {
        double error = TurretConstants.TRACK_SIGN
                * (bearingDeg + TurretConstants.BEARING_OFFSET_DEG);

        // The camera only updates ~30 times a second but this runs much faster, so
        // only recompute the derivative when a new bearing arrives.
        if (error != lastError || !hasLastError) {
            double deltaTime = timer.seconds();
            timer.reset();
            if (hasLastError && deltaTime > 0.001) {
                dTerm = ((error - lastError) / deltaTime) * TurretConstants.kD;
            }
            lastError = error;
            hasLastError = true;
        }

        if (Math.abs(error) < TurretConstants.ANGLE_TOLERANCE_DEG) {
            applyPower(0);
            return;
        }

        double output = error * TurretConstants.kP + dTerm
                + Math.signum(error) * TurretConstants.kS;

        applyPower(Range.clip(
                output,
                -TurretConstants.MAX_TRACK_POWER,
                TurretConstants.MAX_TRACK_POWER));
    }

    public void trackToFieldTarget(
            double targetX,
            double targetY,
            double robotX,
            double robotY,
            double robotHeadingRad) {
        double targetBearing = Math.toDegrees(
                robotHeadingRad - Math.atan2(targetY - robotY, targetX - robotX));
        track(wrapDegrees(targetBearing));
    }

    /** Call when the target is lost. Stops the motor and clears the PD state. */
    public void stop() {
        applyPower(0);
        resetPd();
    }

    private void resetPd() {
        lastError = 0;
        hasLastError = false;
        dTerm = 0;
        timer.reset();
    }

    private void applyPower(double requested) {
        double commanded = requested * TurretConstants.POWER_SIGN;
        double degrees = getDegrees();

        if (degrees >= TurretConstants.MAX_LIMIT_DEG && commanded > 0) {
            commanded = 0;
        } else if (degrees <= TurretConstants.MIN_LIMIT_DEG && commanded < 0) {
            commanded = 0;
        }

        power = commanded;
        motor.setPower(commanded);
    }

    public double getDegrees() {
        return (motor.getCurrentPosition() - startingPosition) / TurretConstants.TICKS_PER_DEGREE;
    }

    public int getTicks() {
        return motor.getCurrentPosition() - startingPosition;
    }

    public double getPower() {
        return power;
    }

    private static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        } else if (wrapped <= -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }
}
