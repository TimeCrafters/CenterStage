package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.util.Range;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class Move extends CyberarmState {
    private final RedCrabMinibot robot;
    private final String groupName, actionName;
    private final double distanceMM, lerpMM_UP, lerpMM_DOWN, maxPower, minPower, toleranceMM;
    private boolean strafe = false;
    private final int timeoutMS;
    private double initialHeadingDegrees = 1024.0;
    public Move(RedCrabMinibot robot, String groupName, String actionName) {
        this.robot = robot;
        this.groupName = groupName;
        this.actionName = actionName;

        this.distanceMM = robot.config.variable(groupName, actionName, "distanceMM").value();
        this.lerpMM_UP = robot.config.variable(groupName, actionName, "lerpMM_UP").value();
        this.lerpMM_DOWN = robot.config.variable(groupName, actionName, "lerpMM_DOWN").value();
        this.toleranceMM = robot.config.variable(groupName, actionName, "toleranceMM").value();

        this.maxPower = robot.config.variable(groupName, actionName, "maxPower").value();
        this.minPower = robot.config.variable(groupName, actionName, "minPower").value();

        this.timeoutMS = robot.config.variable(groupName, actionName, "timeoutMS").value();

        // Validate distance and lerp distances
        if (lerpMM_UP == 0 && lerpMM_DOWN == 0) { return; }

        // --- Good lerp UP distance?
        if (Math.abs(distanceMM) - lerpMM_UP < 0) {
            throw(new RuntimeException("Invalid lerp UP distance"));
        // --- Good lerp DOWN distance?
        } else if (Math.abs(distanceMM) - (lerpMM_UP + lerpMM_DOWN) < 0) {
            throw(new RuntimeException("Invalid lerp distance(s)"));
        }
    }

    @Override
    public void start() {
        initialHeadingDegrees = Utilities.facing(robot.imu);

        if (initialHeadingDegrees > 365.0) {
            throw(new RuntimeException("INVALID Initial IMU value!"));
        }

        if (strafe) {
            robot.frontLeft.resetEncoder();
            robot.frontRight.resetEncoder();
            robot.backLeft.resetEncoder();
            robot.backRight.resetEncoder();

            robot.frontLeft.setPositionTolerance(toleranceMM);
            robot.frontRight.setPositionTolerance(toleranceMM);
            robot.backLeft.setPositionTolerance(toleranceMM);
            robot.backRight.setPositionTolerance(toleranceMM);

            robot.frontLeft.setTargetDistance(distanceMM);
            robot.frontRight.setTargetDistance(-distanceMM);
            robot.backLeft.setTargetDistance(-distanceMM);
            robot.backRight.setTargetDistance(distanceMM);
        } else {
            robot.left.resetEncoder();
            robot.right.resetEncoder();

            robot.left.setPositionTolerance(toleranceMM);
            robot.right.setPositionTolerance(toleranceMM);

            robot.left.setTargetDistance(distanceMM);
            robot.right.setTargetDistance(distanceMM);
        }
    }

    @Override
    public void exec() {
        if (strafe) {
            strafeMove();
        } else {
            tankMove();
        }
    }

    private void tankMove(){
        double travelledDistance = Math.abs(robot.left.getDistance());
        double power = lerpPower(travelledDistance);

        double angleDiff = Utilities.angleDiff(initialHeadingDegrees, Utilities.facing(robot.imu));

        double leftPower = power;
        double rightPower = power;
        // use +10% of power at 7 degrees of error to correct angle
        double correctivePower = Utilities.lerp(0.0, 1.0, angleDiff / 7.0) * (power + power * 0.1);
        if (angleDiff < -0.5) {
            leftPower += correctivePower;
        } else if (angleDiff > 0.5) {
            rightPower += correctivePower;
        }

        robot.left.set(leftPower);
        robot.right.set(rightPower);

        if (robot.left.atTargetPosition() && robot.right.atTargetPosition()) {
            robot.left.set(0);
            robot.right.set(0);

            this.finished();
        }
    }

    private void strafeMove() {
        double travelledDistance = Math.abs(robot.frontLeft.getDistance());
        double power = lerpPower(travelledDistance);

        double angleDiff = Utilities.angleDiff(initialHeadingDegrees, Utilities.facing(robot.imu));

        double frontPower = power;
        double backPower = power;
        // use +10% of power at 7 degrees of error to correct angle
        double correctivePower = Utilities.lerp(0.0, 1.0, angleDiff / 7.0) * (power + power * 0.1);
        if (angleDiff < -0.5) {
            frontPower += correctivePower;
        } else if (angleDiff > 0.5) {
            backPower += correctivePower;
        }

        robot.frontLeft.set(frontPower);
        robot.frontRight.set(-frontPower);
        robot.backLeft.set(-backPower);
        robot.backRight.set(backPower);

        if (robot.frontLeft.atTargetPosition() && robot.backRight.atTargetPosition()) {
            robot.frontLeft.set(0);
            robot.frontRight.set(0);
            robot.backLeft.set(0);
            robot.backRight.set(0);

            this.finished();
        }
    }

    private double lerpPower(double travelledDistance) {
        double lerpPower = maxPower;

        // Ease power up
        if (travelledDistance < lerpMM_UP) { // Not using <= to prevent divide by zero
            lerpPower = Utilities.lerp(minPower, maxPower, Range.clip(travelledDistance / lerpMM_UP, 0.0, 1.0));
            // Cruising power
        } else if (travelledDistance < Math.abs(distanceMM) - lerpMM_DOWN) {
            lerpPower = maxPower;
            // Ease power down
        } else {
            lerpPower = Utilities.lerp(minPower, maxPower, Range.clip( (Math.abs(distanceMM) - travelledDistance) / lerpMM_DOWN, 0.0, 1.0));
        }

        return lerpPower;
    }
}
