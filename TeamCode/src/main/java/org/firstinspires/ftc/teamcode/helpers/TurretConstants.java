package org.firstinspires.ftc.teamcode.helpers;

public class TurretConstants {

    // ---------------- MECHANICAL ----------------

    // 435 RPM Yellow Jacket = 383.6 ticks/rev (312 RPM = 537.7 | 223 RPM = 751.8)
    public static final double GOBILDA_MOTOR_TICKS = 383.6;
    // Driven gear / driving gear (48/20)
    public static final double EXTERNAL_GEAR_RATIO = 2.4;
    public static final double TICKS_PER_DEGREE =
            (GOBILDA_MOTOR_TICKS * EXTERNAL_GEAR_RATIO) / 360.0;

    // Soft limits in degrees from the initialized center
    public static final double MAX_LIMIT_DEG = 180;
    public static final double MIN_LIMIT_DEG = -135;

    // Flip to -1 if positive power turns the turret the wrong way
    public static final double POWER_SIGN = -1.0;

    // ---------------- TRACKING TUNING ----------------

    public static double kP = 0.02;
    public static double kD = 0.0;
    // Minimum power to overcome static friction, added in the direction of the error.
    // Must be non-zero or small errors produce too little power to move the gearbox.
    public static double kS = 0.07;

    // +1 or -1. If the turret runs away from the target (drives to a limit), flip this
    // (press B in the Turret Tracking opmode to flip it live).
    public static double TRACK_SIGN = -1.0;

    // Stop correcting when within this many degrees of the target
    public static double ANGLE_TOLERANCE_DEG = 1.5;
    // Aim offset in degrees, added to the target bearing (shoot left/right of tag center)
    public static double BEARING_OFFSET_DEG = 0.0;

    public static double MAX_TRACK_POWER = 0.6;
    public static double MANUAL_POWER_SCALE = 0.4;

    // ---------------- GOBILDA GLOBAL SHUTTER CAMERA (3122-0004-0001) ----------------
    // OV9782, USB VID 0xC45 / PID 0x366, calibrated by goBILDA at each resolution.
    // If you change the resolution, change the intrinsics to match:
    //   1280x800: 908.758, 908.758, 696.345, 376.979
    //   1280x720: 908.683, 908.683, 706.785, 337.571
    //    800x600: 684.509, 684.509, 444.832, 283.680
    //    640x480: 545.584, 545.584, 350.588, 223.150
    //    320x240: 270.260, 270.260, 173.207, 120.382
    public static final String CAMERA_NAME = "Webcam 1";
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    public static final double CAMERA_FX = 545.584;
    public static final double CAMERA_FY = 545.584;
    public static final double CAMERA_CX = 350.588;
    public static final double CAMERA_CY = 223.150;
}
