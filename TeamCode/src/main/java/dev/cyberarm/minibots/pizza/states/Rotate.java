package dev.cyberarm.minibots.pizza.states;

import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;

public class Rotate extends CyberarmState {
    final private PizzaMinibot robot;
    final private String groupName, actionName;

    final private double heading, velocity, tolerance;
    final private int timeoutMS;

    public Rotate(PizzaMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.heading = robot.config.variable(groupName, actionName, "heading").value();

        this.velocity = robot.config.variable(groupName, actionName, "velocity").value();

        this.tolerance = robot.config.variable(groupName, actionName, "tolerance").value();

        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    public Rotate(PizzaMinibot robot, double heading, double velocity, double tolerance, int timeoutMS) {
        this.groupName = "";
        this.actionName = "";

        this.robot = robot;
        this.heading = heading;
        this.velocity = velocity;
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

        if (robot.angleDiff(robot.facing() + robot.turnRate(), heading) < 0) {
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
