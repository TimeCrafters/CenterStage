package dev.cyberarm.minibots.yellow.states;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.minibots.yellow.YellowMinibot;

public class Move extends CyberarmState {
    final YellowMinibot robot;
    final double leftDistance, rightDistance, leftPower, rightPower;
    public Move(YellowMinibot robot, double leftDistance, double rightDistance, double leftPower, double rightPower) {
        this.robot = robot;

        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    @Override
    public void start() {
        robot.leftFront.setTargetDistance(leftDistance);
        robot.leftBack.setTargetDistance(leftDistance);

        robot.leftFront.setTargetDistance(rightDistance);
        robot.leftBack.setTargetDistance(rightDistance);
    }

    @Override
    public void exec() {
        if (robot.leftFront.atTargetPosition()) {
            robot.leftFront.setVelocity(0);
            robot.leftBack.setVelocity(0);
        }
    }
}
