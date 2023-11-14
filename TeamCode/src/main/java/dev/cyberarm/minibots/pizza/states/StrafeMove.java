package dev.cyberarm.minibots.pizza.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;

public class StrafeMove extends CyberarmState {
    final private PizzaMinibot robot;
    final private String groupName, actionName;

    final private double distanceMM, velocity;
    final private int tolerance, timeoutMS;

    public StrafeMove(PizzaMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.distanceMM = robot.config.variable(groupName, actionName, "distanceMM").value();

        this.velocity = robot.config.variable(groupName, actionName, "velocity").value();

        this.tolerance = robot.config.variable(groupName, actionName, "tolerance").value();
        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    public StrafeMove(PizzaMinibot robot, double distanceMM, double velocity, int tolerance, int timeoutMS) {
        this.groupName = "";
        this.actionName = "";

        this.robot = robot;
        this.distanceMM = distanceMM;
        this.velocity = velocity;
        this.tolerance = tolerance;
        this.timeoutMS = timeoutMS;
    }

    @Override
    public void start() {
        robot.leftFront.setTargetDistance(distanceMM);

        robot.left.setPositionTolerance(tolerance);
        robot.right.setPositionTolerance(tolerance);

        double motorVelocity = (distanceMM < 0 ? velocity * -1 : velocity);

        robot.leftFront.set(motorVelocity);
        robot.rightFront.set(-motorVelocity);

        robot.leftBack.set(-motorVelocity);
        robot.rightBack.set(motorVelocity);
    }

    @Override
    public void exec() {
        if (robot.left.atTargetPosition() || Math.abs(robot.leftFront.getDistance()) >= Math.abs(distanceMM) ||  runTime() >= timeoutMS) {
            robot.left.set(0);
            robot.right.set(0);
            setHasFinished(true);
        }
    }
}
