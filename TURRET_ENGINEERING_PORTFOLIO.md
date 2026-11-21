# Engineering Portfolio: Autonomous Turret and Target Tracking

## Project Overview

Our team created an independently rotating turret that allows the shooter to aim toward the alliance HIVE while the robot continues driving across the field. Before the turret was developed, the robot would need to turn the entire drivetrain to point toward the goal. That approach limited our ability to collect game pieces, avoid opponents, and shoot while moving. The turret separates movement and aiming, allowing the drivetrain and shooter to perform independently. The project included the mechanical gear system, encoder position tracking, target calculations, automatic control, manual backup control, and a system for tuning the aiming response.

![Turret system overview](engineering-portfolio/images/01-turret-system-overview.png)

## Problem and Engineering Requirements

The main problem was that a fixed-direction shooter requires the robot to face the goal for every shot. Our team needed a system that could continue aiming while the robot strafed or rotated. The turret needed to rotate left and right, know its position, stop before reaching its mechanical limits, aim toward a moving robot-relative target, and remain controllable if localization or automatic aiming was not accurate. It also needed to be usable for both alliances and allow the driver to switch quickly between automatic and manual operation.

We established several engineering requirements. The turret had to use feedback rather than relying only on a timed motor command. It had to have software safety limits to protect the gears and wiring. It had to respond quickly without repeatedly overshooting the target. It also needed a backup method of aiming in case the robot’s location became uncertain. Finally, it had to be designed so gains and direction settings could be adjusted during testing without modifying every part of the match strategy.

## Mechanical Design

The turret turn motor is a 435 RPM goBILDA Yellow Jacket motor with an encoder. The encoder is important because it allows the control system to measure how far the turret has rotated instead of assuming that a motor command always produces the intended movement. The shooter motor is configured to operate at approximately 6,000 RPM, so it can spin the shooter wheels quickly enough to launch game pieces toward the HIVE. The turn motor and shooter motor have separate jobs: the turn motor positions the turret, while the shooter motor provides the velocity required for the ball to travel to the target.

The turret turn motor is driven through an external 48-to-20 gear reduction, producing a 2.4-to-1 gear ratio. The motor gear is the 20-tooth gear, and the turret gear is the 48-tooth gear. This means the turret rotates 360 degrees only after the motor completes 2.4 revolutions.

![Gear reduction and encoder system](engineering-portfolio/images/02-gear-and-encoder.png)

The encoder provides approximately 383.6 counts for each motor revolution. Because of the external gear reduction, the software calculates that there are approximately 2.56 encoder counts per degree of turret rotation. We used this value to convert motor encoder position into turret angle. The turret angle is measured relative to the position where the mechanism is initialized, so the robot should start with the turret in a consistent position after each restart.

This gear ratio was chosen because it provides enough torque to rotate the mechanism while allowing the encoder to provide a measurable and controllable amount of position feedback. A direct-drive system would require a gear ratio that reduces torque and could make the motor work harder. A much larger reduction could provide more torque but would also create more friction and require greater accuracy from the encoder. The 48-to-20 reduction offered a practical balance between torque, position resolution, and compact mechanical packaging.

## Turret Position and Soft Limits

The program converts the encoder difference from the starting position into degrees and compares that angle with software-defined limits. The normal maximum angle is +180 degrees and the minimum angle is -135 degrees. These values create a safe working range of 315 degrees from the initialized center position. Before sending motor power, the program checks whether the turret is already at or beyond a limit and whether the requested power would move it farther outward.

![Turret range and soft limits](engineering-portfolio/images/03-turret-soft-limits.png)

For example, if the turret reaches +180 degrees, additional positive power is removed so the mechanism cannot move farther beyond that limit. If the turret rotates toward the center, that power is allowed again. The same process is used for the -135-degree limit. This creates a soft limit that protects the mechanism from motor commands that would otherwise continue pushing it into a hard stop.

The software limits do not replace a physical end stop or mechanical inspection. Instead, they provide a second layer of protection that reduces stress on gears, wire routing, and mounting parts. The encoder position is reset whenever the robot restarts, so the soft-limit values are reliable only while the turret begins from a repeatable initialized position.

## Target Selection

The turret uses Pedro Pathing localization to estimate the robot’s position and heading on the field. It combines the robot’s X position, Y position, and heading with fixed HIVE coordinates. The system chooses a red or blue target depending on alliance and chooses a scoring-side or audience-side target depending on whether the robot is in the corresponding field half.

For red alliance operation, the scoring and audience targets are defined using fixed field coordinates. The blue alliance target uses a mirrored position across the field midline. The turret does not need separate alliance aiming code for this calculation because the target coordinates are selected by the software. This makes the same TeleOp structure usable for both alliances.

The code compares the direction from the robot to the target with the direction the robot is currently facing. It then calculates a target bearing for the turret and wraps the result to an angle between approximately -180 and +180 degrees. Angle wrapping is important because a target that is 350 degrees to the left may only require a 10-degree clockwise rotation. This prevents the turret from making a large, unnecessary rotation and helps it reach the target more efficiently.

![Field localization and target tracking](engineering-portfolio/images/04-field-tracking.png)

## Automatic Tracking Control

The turret uses a PD controller combined with static-friction compensation. The proportional term creates more motor power when the turret is farther from the target and less power as it approaches the target. The derivative term uses the changing amount of angle error to reduce overshoot and oscillation. The static-friction term adds a small amount of power in the direction of the error so the gearbox can overcome the resistance needed to begin moving.

The control response is based on this general relationship: proportional control moves the turret toward the target, derivative control responds to turret motion and reduces sudden overshooting, and static-friction compensation supplies enough starting force for small errors. The initial proportional gain is 0.02, the derivative gain is 0.0, and the static-friction power is 0.07. These values provide a starting point for tuning rather than being assumed to be perfect for every battery level or mechanical condition.

The system stops correction when the angle error is less than 1.5 degrees. This tolerance prevents the motor from repeatedly correcting extremely small errors and helps avoid continuous vibration near the target. The maximum automatic tracking power is limited to 0.6, so the turret cannot rapidly slam toward a target or use excessive torque. These limits provide a safer and more stable response than allowing full motor power whenever the turret detects an angle error.

## Direction Correction and Live Tuning

The control system includes separate settings for motor power direction and tracking direction. The motor power direction can be changed if positive motor power rotates the turret physically in the wrong direction. The tracking direction can be changed if the turret moves away from the target instead of toward it. We also included a control that reverses the tracking direction while testing.

During tuning, proportional gain can be increased or decreased, derivative gain can be adjusted in smaller steps, and static-friction compensation can be raised or lowered. Holding a fine-adjustment button makes each change smaller. This allows the team to test several values in one session without repeatedly rebuilding and redeploying code.

Live tuning has significant advantages for testing because gains can be adjusted as the turret is observed. It also helps separate mechanical problems from control problems. For example, if the turret moves in the wrong direction, the tracking direction can be reversed. If the turret stalls with small errors, kS can be increased. If the turret overshoots, kD or kP can be changed. This makes the debugging process more systematic.

The live values reset when the robot restarts, so tuning is only temporary unless the final values are copied into the permanent configuration. This is a safe behavior because it prevents an unverified tuning session from becoming the only default setup. The final gain values must be tested repeatedly before they are considered reliable.

## Manual Backup Control

The turret can switch from automatic tracking to manual control. The driver can use the left and right bumpers or a gamepad stick to rotate the mechanism. Manual power is limited to 40 percent of full motor power so it remains controllable and less likely to cause a sudden impact.

Manual control was included as a backup and testing feature. It allows the driver to aim the turret when localization may be inaccurate or to operate the robot if automatic tracking is behaving incorrectly. It also gives the team a method of testing the mechanical range and checking whether the positive and negative directions match the control labels.

The disadvantage of manual control is that the driver must divide attention among driving, intake, shooting, and turret aiming. Manual control may be less accurate while the robot is moving and may not respond as consistently as automatic tracking. However, having a backup mode means the robot remains usable even if the automatic target calculation is no longer reliable.

## Advantages of the Turret System

The greatest advantage of the turret is that it allows the shooter to aim independently of the drivetrain. The robot can collect game pieces or move around opponents while continuing to point the shooter toward the HIVE. This is faster and more flexible than stopping and turning the entire robot. It also gives the driver a wider field of movement because facing the goal is no longer required for every shot.

The encoder gives measurable position feedback, while the software limits protect the turret from excessive rotation. The target calculation updates based on field localization rather than requiring the robot to receive a visual target at every moment. The automatic controller reduces the need for continuous manual input, and the derivative term helps prevent unnecessary overshooting. The system also includes manual control and live tuning as additional safety and testing features.

## Disadvantages and Limitations

The turret has a limited range of approximately 315 degrees, so it cannot aim continuously in every direction. If the driver moves the robot into a position that would require a rotation outside the safe range, the system may stop at a soft limit before the shooter is aimed directly at the target. This is why the allowed range must balance aiming flexibility against mechanical and wire protection.

The encoder is also a relative position sensor. Its starting value is reset each time the system begins. If the turret is not in a consistent initial position, the reported angle may not match the physical direction of the turret. The mechanical system must therefore have a repeatable starting position or a separate homing method for more accurate tracking.

Automatic aiming also depends on localization accuracy. If the robot’s estimated position or heading becomes incorrect, the turret will calculate a bearing toward the wrong location. The turret will still move correctly relative to the incorrect estimated data, but the shot may miss. This means the turret cannot be more accurate than the robot’s position estimate.

The PD controller also has limitations. A proportional gain that is too high can cause oscillation. Too little proportional gain can make the turret slow or unable to overcome resistance. A large derivative gain can amplify sensor noise or make the system feel jerky. Static-friction compensation can also push the turret in the wrong direction if the tracking sign is incorrect. Therefore, tuning must be done carefully through measured testing.

## Alternatives Considered

One alternative was to use no turret and simply rotate the drivetrain toward the goal. This would reduce mechanical complexity, but it would limit the robot’s mobility and require the driver to choose between driving and aiming. It would also make strafing and obstacle avoidance more difficult because the robot would need to maintain a specific heading.

Another alternative was to use direct AprilTag tracking with a camera. This could provide a visual target and potentially correct localization errors. However, it would require reliable detection from every shooting position and could be affected by lighting, occlusion, field traffic, and camera position. The current localization-based design is less dependent on direct camera visibility and can calculate the target direction even when the target is not in the image.

A third alternative was pure proportional control. This would be simpler to code but could stall at very small errors or continuously overshoot. Our PD-plus-static-friction approach gives the system faster starting motion, controlled slowing near the target, and a better opportunity to hold a stable angle.

A continuous-rotation turret was also considered, but it would add wire-management, alignment, and mechanical complexity. It could provide a complete 360-degree range, but it would also require a way to prevent cables from twisting and could make soft-limit and encoder position control more complicated. For our robot, the 315-degree working range provides enough flexibility without unnecessary rotational hardware.

## Pros and Cons

![Turret advantages and tradeoffs](engineering-portfolio/images/05-pros-and-cons.png)

### Advantages

- The turret aims independently, allowing the drivetrain to continue driving, collecting, defending, or avoiding other robots.
- The encoder gives measurable turret position feedback instead of using an estimated position based only on motor command time.
- The 48-to-20 gear reduction gives the 435 RPM turn motor useful mechanical advantage and controllable position resolution.
- The approximately 6,000 RPM shooter motor provides the flywheel speed needed to launch game pieces toward the HIVE.
- Software limits prevent the turret from continuing outward at its maximum or minimum position.
- The controller slows the turret as it approaches the target and includes a minimum direction-based power to overcome static friction.
- Manual control provides a reliable backup if localization or tracking is temporarily inaccurate.
- Live tuning allows the team to test and improve gain values without rebuilding the entire operating system.
- The system works for both alliances and for both field halves.

### Disadvantages

- The turret is limited to approximately 315 degrees and cannot continuously rotate through 360 degrees.
- Accuracy depends on the reliability of the robot’s localization.
- Localization drift can make the turret calculate a direction toward the wrong position.
- The encoder is reset at startup, so the turret must begin from a repeatable position.
- A target in a position outside the mechanical range may prevent the turret from aiming directly at it.
- Gains such as kP, kD, and kS require testing and may change with battery voltage or friction.
- Too much kP or kD can create overshoot or vibration.
- Manual control requires additional attention from the driver.
- The gear reduction adds friction, alignment requirements, and moving parts that require inspection and maintenance.

## Testing and Verification

The system should be tested at several field positions. A basic test should measure whether the turret reaches the same angle repeatedly when starting from the same position. The team should test it while the robot is stationary, while driving straight, and while rotating or strafing. It should also be tested toward both alliance targets and across both field halves.

![Test evidence and photo plan](engineering-portfolio/images/06-test-photo-plan.png)

The first test should verify the mechanical and electrical signs. The positive motor direction, encoder direction, and tracking direction must all agree with the intended turret movement. The next test should verify the soft limits by slowly moving toward both ends of the range. The test should confirm that the motor stops before reaching the hard stop and that power returns when the turret moves inward.

For controller tuning, the team should begin with kP and kS, then adjust kD only after the turret has a stable mechanical response. The test should measure whether the turret reaches 1.5 degrees of the target, whether it overshoots, and whether it oscillates. kP should be increased until response is fast but not unstable. kS should be raised only if the gearbox does not move for small errors and lowered if it creates movement at the target. kD should be adjusted if overshoot or oscillation is observed.

The team should record results for each test, including the initial gain values, final gain values, test position, whether localization was accurate, battery condition, overshoot, settling time, and whether the shot was successful. This data would allow the team to determine whether changes improve performance consistently rather than only in one test position.

## Why This Design Is Better

This turret is better for our robot because it separates movement and aiming. The drive team can continue collecting, defending, and positioning while the turret keeps adjusting toward the target. It is more flexible than a fixed shooter and more practical than requiring the drivetrain to align perfectly with the goal.

The design is also safer and more controllable because of its encoder feedback, software limits, limited tracking power, angle tolerance, and manual override. It is more efficient to test because gains can be adjusted while the robot is operating. It is also scalable because the same control structure can support both alliances and both scoring positions.

The greatest improvement is not simply that the turret rotates. The improvement is that the robot gains a second degree of freedom. Instead of turning the entire mechanism, the system can aim the shooter separately while localization continuously calculates the required direction. That additional degree of freedom should improve mobility, shot consistency, and driver flexibility during matches.
