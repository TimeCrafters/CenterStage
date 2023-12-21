package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class Rotate extends CyberarmState {
    final private RedCrabMinibot robot;
    final private String groupName, actionName;

    final private double maxVelocityMM, minVelocityMM, lerpDegrees, headingDegrees, toleranceDegrees;
    final private int timeoutMS;
    private boolean commitToRotation = false;

    public Rotate(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.maxVelocityMM = robot.config.variable(groupName, actionName, "maxVelocityMM").value();
        this.minVelocityMM = robot.config.variable(groupName, actionName, " minVelocityMM").value();

        this.lerpDegrees = robot.config.variable(groupName, actionName, "lerpDEGREES").value();
        this.headingDegrees = robot.config.variable(groupName, actionName, "headingDEGREES").value();
        this.toleranceDegrees = robot.config.variable(groupName, actionName, "toleranceDEGREES").value();

        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    @Override
    public void exec() {
        double angleDiff = Utilities.angleDiff(Utilities.facing(robot.imu), headingDegrees);

        if (Math.abs(angleDiff) <= toleranceDegrees || runTime() >= timeoutMS) {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            this.finished();

            return;
        }

        double velocityMM = Utilities.lerp(minVelocityMM, maxVelocityMM, Range.clip(Math.abs(angleDiff) / lerpDegrees, 0.0, 1.0));
        double velocity = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                velocityMM);

        if (!commitToRotation) {
            if (angleDiff < 0) {
                robot.frontLeft.setVelocity(-velocity);
                robot.frontRight.setVelocity(velocity);
                robot.backLeft.setVelocity(-velocity);
                robot.backRight.setVelocity(velocity);
            } else {
                robot.frontLeft.setVelocity(velocity);
                robot.frontRight.setVelocity(-velocity);
                robot.backLeft.setVelocity(velocity);
                robot.backRight.setVelocity(-velocity);
            }
        }

        commitToRotation = Math.abs(angleDiff) > 170;
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Robot Heading", Utilities.facing(robot.imu));
        engine.telemetry.addData("Robot Target Heading", headingDegrees);
        engine.telemetry.addData("Robot Angle Diff", Utilities.angleDiff(Utilities.facing(robot.imu), headingDegrees));
        engine.telemetry.addData("Robot Turn Rate", Utilities.turnRate(robot.imu));
    }
}
