package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.engine.V2.Vector2D;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class MoveToCoordinate extends CyberarmState {
    private final RedCrabMinibot robot;
    private final String groupName, actionName;
    private final boolean stopDrivetrain;
    private final int timeoutMS;
    private final double targetAngleDegrees, targetAngleToleranceDegrees, minDistanceMM;
    private final double maxVelocityMM, minVelocityMM, lerpMM_UP, lerpMM_DOWN, lerpDegrees;
    private final Vector2D targetPosMM;
    private Vector2D robotInitialPosMM = new Vector2D(), robotPosMM = new Vector2D();
    private Vector2D direction = new Vector2D();
    private double distanceFromTargetMM = 0;
    private double velocity = 0;
    private double angleDiffDegrees = 0;

    public MoveToCoordinate(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();
        this.stopDrivetrain = robot.config.variable(groupName, actionName, "stopDrivetrain").value();

        this.targetAngleToleranceDegrees = robot.config.variable(groupName, actionName, "targetAngleToleranceDEGREES").value();
        this.targetAngleDegrees = robot.config.variable(groupName, actionName, "targetAngleDEGREES").value();

        this.lerpDegrees = robot.config.variable(groupName, actionName, "lerpDEGREES").value();
        this.lerpMM_UP = robot.config.variable(groupName, actionName, "lerpMM_UP").value();
        this.lerpMM_DOWN = robot.config.variable(groupName, actionName, "lerpMM_DOWN").value();

        this.maxVelocityMM = robot.config.variable(groupName, actionName, "maxVelocityMM").value();
        this.minVelocityMM = robot.config.variable(groupName, actionName, "minVelocityMM").value();

        this.minDistanceMM = robot.config.variable(groupName, actionName, "minDistanceMM").value();
        String targetPosMM = robot.config.variable(groupName, actionName, "targetPosMM").value();
        String[] targetPos = targetPosMM.split("x");
        this.targetPosMM = new Vector2D(Double.parseDouble(targetPos[0]), Double.parseDouble(targetPos[1]));
    }

    @Override
    public void start() {
        this.robotInitialPosMM = new Vector2D(RedCrabMinibot.localizer.xMM(), RedCrabMinibot.localizer.yMM());
    }

    @Override
    public void exec() {
        this.robotPosMM = new Vector2D(RedCrabMinibot.localizer.xMM(), RedCrabMinibot.localizer.yMM());
        this.direction = targetPosMM.minus(robotPosMM).normalize();
        this.distanceFromTargetMM = robotPosMM.distance(this.targetPosMM);
        double newAngleDiffDegrees = Utilities.angleDiff(Utilities.facing(robot.imu), targetAngleDegrees);
        // FIXME: Test this!
        // Ignore new angle diff since it appears to be a numeric sign flip and not a useful value (prevent toggling shortest rotation decision.)
        if (!(Math.abs(newAngleDiffDegrees) - Math.abs(this.angleDiffDegrees) <= 2.0 && Math.abs(newAngleDiffDegrees) >= 178.0))
            this.angleDiffDegrees = newAngleDiffDegrees;


        boolean rotationGood = Utilities.isBetween(
                angleDiffDegrees,
                targetAngleDegrees - targetAngleToleranceDegrees,
                targetAngleDegrees + targetAngleToleranceDegrees);

        if ((distanceFromTargetMM <= minDistanceMM && rotationGood) || runTime() >= timeoutMS) {
            if (stopDrivetrain) {
                robot.frontLeft.setVelocity(0);
                robot.frontRight.setVelocity(0);
                robot.backLeft.setVelocity(0);
                robot.backRight.setVelocity(0);
            }

            finished();
            return;
        }

        drivetrain(direction);
    }

    private void drivetrain(Vector2D direction) {
        double rotationStrength = Utilities.lerp(
                minVelocityMM,
                maxVelocityMM,
                Range.clip(Math.abs(angleDiffDegrees) / lerpDegrees, 0.0, 1.0));
        if (angleDiffDegrees < 0)
            rotationStrength *= -1;

        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric

        double y = direction.x(); // robot forward is in the X axis
        double x = direction.y(); // robot side to side is on the Y axis
        double rx = rotationStrength; //engine.gamepad1.right_stick_x;

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        velocity = lerpVelocity();

        robot.frontLeft.setVelocity(frontLeftPower * velocity);
        robot.backLeft.setVelocity(backLeftPower * velocity);
        robot.frontRight.setVelocity(frontRightPower * velocity);
        robot.backRight.setVelocity(backRightPower * velocity);
    }

    private double lerpVelocity() {
        double distanceFromInitialPosMM = robotInitialPosMM.distance(robotPosMM);
        double lerpVelocity;

        // Ease power up
        if (distanceFromInitialPosMM < lerpMM_UP) {
            lerpVelocity = Utilities.lerp(
                    minVelocityMM,
                    maxVelocityMM,
                    Range.clip(
                            distanceFromInitialPosMM / lerpMM_UP, 0.0, 1.0));
            // Cruising power
        } else if (distanceFromTargetMM > lerpMM_DOWN) {
            lerpVelocity = maxVelocityMM;
            // Ease power down
        } else {
            lerpVelocity = Utilities.lerp(
                    minVelocityMM,
                    maxVelocityMM,
                    Range.clip(
                            distanceFromTargetMM / lerpMM_DOWN, 0.0, 1.0));
        }

        return lerpVelocity;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Current Position MM", "X: %.2f Y: %.2f", robotPosMM.x(), robotPosMM.y());
        engine.telemetry.addData("Target Position MM", "X: %.2f Y: %.2f", targetPosMM.x(), targetPosMM.y());
        engine.telemetry.addData("Move NORMAL", "X: %.2f Y: %.2f", direction.x(), direction.y());
        engine.telemetry.addData("Distance MM", "%.2fmm", distanceFromTargetMM);
        engine.telemetry.addData("Angle Diff DEGREES", "%.2f degrees", angleDiffDegrees);
//        engine.telemetry.addData("Velocity", "%.2f T/s", velocity);
//        engine.telemetry.addData("LERP DOWN", "%.2f (%.2fmm)", distanceFromTargetMM / lerpMM_DOWN, distanceFromTargetMM);
    }
}
