package dev.cyberarm.minibots.yellow.states;

import com.qualcomm.robotcore.util.Range;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class Rotate extends CyberarmState {
    final private YellowMinibot robot;
    final private String groupName, actionName;

    final private double heading, maxVelocity, minVelocity, tolerance;
    final private int timeoutMS;

    public Rotate(YellowMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.heading = robot.config.variable(groupName, actionName, "heading").value();

        this.minVelocity = robot.config.variable(groupName, actionName, "minVelocity").value();
        this.maxVelocity = robot.config.variable(groupName, actionName, "maxVelocity").value();

        this.tolerance = robot.config.variable(groupName, actionName, "tolerance").value();

        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    public Rotate(YellowMinibot robot, double heading, double maxVelocity, double minVelocity, double tolerance, int timeoutMS) {
        this.groupName = "";
        this.actionName = "";

        this.robot = robot;
        this.heading = heading;
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        this.tolerance = tolerance;
        this.timeoutMS = timeoutMS;
    }

    @Override
    public void start() {
    }

    @Override
    public void exec() {
        if (Math.abs(robot.angleDiff(robot.facing(), heading)) <= tolerance) {
            robot.left.set(0);
            robot.right.set(0);

            setHasFinished(true);

            return;
        }

        if (runTime() >= timeoutMS) {
            robot.left.set(0);
            robot.right.set(0);

            setHasFinished(true);
        }

        double angleDiff = robot.angleDiff(robot.facing() + robot.turnRate(), heading);
        double velocity = robot.lerp(minVelocity, maxVelocity, Range.clip(Math.abs(angleDiff) / 90.0, 0.0, 1.0));

        if (angleDiff > 0) {
            robot.left.set(-velocity);
            robot.right.set(velocity);
        } else {
            robot.left.set(velocity);
            robot.right.set(-velocity);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Robot Heading", robot.facing());
        engine.telemetry.addData("Robot Target Heading", heading);
        engine.telemetry.addData("Robot Angle Diff", robot.angleDiff(robot.facing() + robot.turnRate(), heading));
        engine.telemetry.addData("Robot Turn Rate", robot.turnRate());
    }
}
