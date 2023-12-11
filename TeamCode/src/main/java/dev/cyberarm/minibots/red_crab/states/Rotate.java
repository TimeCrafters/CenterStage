package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.util.Range;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class Rotate extends CyberarmState {
    final private RedCrabMinibot robot;
    final private String groupName, actionName;

    final private double maxPower, minPower, lerpDegrees, headingDegrees, toleranceDegrees;
    final private int timeoutMS;

    public Rotate(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.minPower = robot.config.variable(groupName, actionName, "minPower").value();
        this.maxPower = robot.config.variable(groupName, actionName, "maxPower").value();

        this.lerpDegrees = robot.config.variable(groupName, actionName, "lerpDEGREES").value();
        this.headingDegrees = robot.config.variable(groupName, actionName, "headingDEGREES").value();
        this.toleranceDegrees = robot.config.variable(groupName, actionName, "toleranceDEGREEES").value();

        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    @Override
    public void exec() {
        double angleDiff = Utilities.angleDiff(Utilities.facing(robot.imu) + Utilities.turnRate(robot.imu), headingDegrees);

        if (Math.abs(angleDiff) <= toleranceDegrees || runTime() >= timeoutMS) {
            robot.left.set(0);
            robot.right.set(0);

            this.finished();

            return;
        }

        double power = Utilities.lerp(minPower, maxPower, Range.clip(Math.abs(angleDiff) / lerpDegrees, 0.0, 1.0));

        if (angleDiff > 0) {
            robot.left.set(-power);
            robot.right.set(power);
        } else {
            robot.left.set(power);
            robot.right.set(-power);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Robot Heading", Utilities.facing(robot.imu));
        engine.telemetry.addData("Robot Target Heading", headingDegrees);
        engine.telemetry.addData("Robot Angle Diff", Utilities.angleDiff(Utilities.facing(robot.imu) + Utilities.turnRate(robot.imu), headingDegrees));
        engine.telemetry.addData("Robot Turn Rate", Utilities.turnRate(robot.imu));
    }
}
