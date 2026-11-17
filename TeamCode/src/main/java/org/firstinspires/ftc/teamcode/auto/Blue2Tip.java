package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helpers.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.pedro.Constants;

@Autonomous(name = "Blue", group = "Autonomous")
public class Blue2Tip extends LinearOpMode {

    private Follower follower;
    private RobotHardware robotHardware;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorY(70.25).mirrorY(70);

    private final Pose start = poseFactory.of(8, 11, 90+180);
    private final Pose path1 = poseFactory.of(7.9809, 8.0191, 90+180);
    private final Pose shoot2 = poseFactory.of(28.7902, 120.7993, 80);
    private final Pose point3 = poseFactory.of(12, 47, 0);
    private final Pose point4 = poseFactory.of(7.003, 119.6526, 94);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, shoot2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.init();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        robotHardware.left_intake.setPower(1);
        robotHardware.right_intake.setPower(1);
        robotHardware.intakeMotor.setPower(1);
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path shoot2() {
        return Paths.line(path1, shoot2).tangent();
    }

    public Path path3() {
        return Paths.line(shoot2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }
}
