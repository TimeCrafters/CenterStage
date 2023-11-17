package dev.cyberarm.minibots.yellow.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.pizza.PizzaMinibot;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class TankMove extends CyberarmState {
    final private YellowMinibot robot;
    final private String groupName, actionName;

    final private double leftDistanceMM, rightDistanceMM;
    final private double leftVelocity, rightVelocity;
    final private int tolerance, timeoutMS;

    public TankMove(YellowMinibot robot, String groupName, String actionName) {
        this.robot = robot;

        this.groupName = groupName;
        this.actionName = actionName;

        this.leftDistanceMM = robot.config.variable(groupName, actionName, "leftDistanceMM").value();
        this.rightDistanceMM = robot.config.variable(groupName, actionName, "rightDistanceMM").value();

        this.leftVelocity = robot.config.variable(groupName, actionName, "leftVelocity").value();
        this.rightVelocity = robot.config.variable(groupName, actionName, "rightVelocity").value();

        this.tolerance = robot.config.variable(groupName, actionName, "tolerance").value();
        this.timeoutMS  = robot.config.variable(groupName, actionName, "timeoutMS").value();
    }

    public TankMove(YellowMinibot robot, double leftDistanceMM, double rightDistanceMM, double leftVelocity, double rightVelocity, int tolerance, int timeoutMS) {
        this.groupName = "";
        this.actionName = "";

        this.robot = robot;
        this.leftDistanceMM = leftDistanceMM;
        this.rightDistanceMM = rightDistanceMM;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.tolerance = tolerance;
        this.timeoutMS = timeoutMS;
    }

    @Override
    public void start() {
        robot.left.resetEncoder();
        robot.right.resetEncoder();

        robot.left.setTargetDistance(leftDistanceMM);
        robot.right.setTargetDistance(rightDistanceMM);

        robot.left.setPositionTolerance(tolerance);
        robot.right.setPositionTolerance(tolerance);

        robot.left.set(leftVelocity);
        robot.right.set(rightVelocity);
    }

    @Override
    public void exec() {
        if (robot.left.atTargetPosition() || Math.abs(robot.leftFront.getDistance()) >= Math.abs(leftDistanceMM)) {
            robot.left.set(0);
        }
        if (robot.right.atTargetPosition() || Math.abs(robot.rightFront.getDistance()) >= Math.abs(leftDistanceMM)) {
            robot.right.set(0);
        }

        if (
                (robot.left.atTargetPosition() && robot.right.atTargetPosition()) ||
                (Math.abs(robot.leftFront.getDistance()) >= Math.abs(leftDistanceMM) && robot.right.atTargetPosition() || Math.abs(robot.rightFront.getDistance()) >= Math.abs(leftDistanceMM))  ||
                runTime() >= timeoutMS) {
            robot.left.set(0);
            robot.right.set(0);
            setHasFinished(true);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine();

        engine.telemetry.addData("Left distance", robot.leftFront.getDistance());
        engine.telemetry.addData("Left position", robot.left.getPositions().get(0));
        engine.telemetry.addData("Left speed", robot.left.getSpeeds().get(0));
        engine.telemetry.addData("Left velocity", robot.left.getVelocity());
        engine.telemetry.addLine();

        engine.telemetry.addData("Right distance", robot.rightFront.getDistance());
        engine.telemetry.addData("Right position", robot.right.getPositions().get(0));
        engine.telemetry.addData("Right speed", robot.right.getSpeeds().get(0));
        engine.telemetry.addData("Right velocity", robot.right.getVelocity());
    }
}
