package dev.cyberarm.minibots.red_crab.states;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.engine.V2.Vector2D;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class MoveToCoordinate extends CyberarmState {
    private final RedCrabMinibot robot;
    private final String groupName, actionName;
    private final boolean stopDrivetrain;
    private final int timeoutMS;
    private final double targetAngleDegrees, angleToleranceDegrees, minDistanceMM, targetX_MM, targetY_MM;
    private final Vector2D targetPosMM;

    public MoveToCoordinate(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();
        this.stopDrivetrain = robot.config.variable(groupName, actionName, "stopDrivetrain").value();
        this.angleToleranceDegrees = robot.config.variable(groupName, actionName, "angleToleranceDEGREES").value();
        this.targetAngleDegrees = robot.config.variable(groupName, actionName, "targetAngleDEGREES").value();
        this.minDistanceMM = robot.config.variable(groupName, actionName, "minDistanceMM").value();
        String targetPosMM = robot.config.variable(groupName, actionName, "targetPosMM").value();
        String[] targetPos = targetPosMM.split("x");
        this.targetX_MM = Double.parseDouble(targetPos[0]);
        this.targetY_MM = Double.parseDouble(targetPos[1]);

        this.targetPosMM = new Vector2D(this.targetX_MM, this.targetY_MM);
    }

    @Override
    public void exec() {
        Vector2D robotPosMM = new Vector2D(RedCrabMinibot.localizer.xMM(), RedCrabMinibot.localizer.yMM());
        Vector2D direction = targetPosMM.minus(robotPosMM).normalize();
        double distanceMM = robotPosMM.distance(this.targetPosMM);

        if (distanceMM <= minDistanceMM || runTime() >= timeoutMS) {
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
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric

        double y = direction.x(); // robot forward is in the X axis
        double x = direction.y(); // robot side to side is on the Y axis
        // FIXME: Dynamically set rotation as needed to achieve target heading using shortest rotation
        double rx = 0;//engine.gamepad1.right_stick_x;

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

        double maxVelocity = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                RedCrabMinibot.DRIVETRAIN_VELOCITY_MAX_MM);
        double slowVelocity = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                RedCrabMinibot.DRIVETRAIN_VELOCITY_SLOW_MM);
        // FIXME: Lerp up/down when starting move and getting close to target
        double velocity = slowVelocity; //robotSlowMode ? slowVelocity : maxVelocity;

        robot.frontLeft.setVelocity(frontLeftPower * velocity);
        robot.backLeft.setVelocity(backLeftPower * velocity);
        robot.frontRight.setVelocity(frontRightPower * velocity);
        robot.backRight.setVelocity(backRightPower * velocity);
    }

    @Override
    public void telemetry() {
        Vector2D robotPosMM = new Vector2D(RedCrabMinibot.localizer.xMM(), RedCrabMinibot.localizer.yMM());
        Vector2D direction = targetPosMM.minus(robotPosMM).normalize();

        engine.telemetry.addData("Current Position MM", "X: %.2f Y: %.2f", robotPosMM.x(), robotPosMM.y());
        engine.telemetry.addData("Target Position MM", "X: %.2f Y: %.2f", targetPosMM.x(), targetPosMM.y());
        engine.telemetry.addData("Move NORMAL", "X: %.2f Y: %.2f", direction.x(), direction.y());
        engine.telemetry.addData("Distance MM", "%.2fmm", robotPosMM.distance(targetPosMM));
    }
}
