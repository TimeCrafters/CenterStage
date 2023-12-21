package dev.cyberarm.minibots.red_crab.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.cyberarm.engine.V2.CyberarmState;
import dev.cyberarm.engine.V2.Utilities;
import dev.cyberarm.minibots.red_crab.RedCrabMinibot;

public class Move extends CyberarmState {
    private final RedCrabMinibot robot;
    private final String groupName, actionName;
    private final double distanceMM, lerpMM_UP, lerpMM_DOWN, maxVelocityMM, minVelocityMM, toleranceMM;
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

        this.maxVelocityMM = robot.config.variable(groupName, actionName, "maxVelocityMM").value();
        this.minVelocityMM = robot.config.variable(groupName, actionName, "minVelocityMM").value();

        this.strafe = robot.config.variable(groupName, actionName, "strafe").value();

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

        int toleranceTicks = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                toleranceMM);
        int distanceTicks = Utilities.unitToTicks(
                RedCrabMinibot.DRIVETRAIN_MOTOR_TICKS_PER_REVOLUTION,
                RedCrabMinibot.DRIVETRAIN_GEAR_RATIO,
                RedCrabMinibot.DRIVETRAIN_WHEEL_DIAMETER_MM,
                DistanceUnit.MM,
                toleranceMM);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontLeft.setTargetPositionTolerance(toleranceTicks);
        robot.frontRight.setTargetPositionTolerance(toleranceTicks);
        robot.backLeft.setTargetPositionTolerance(toleranceTicks);
        robot.backRight.setTargetPositionTolerance(toleranceTicks);

        if (strafe) {
            robot.frontLeft.setTargetPosition(distanceTicks);
            robot.frontRight.setTargetPosition(-distanceTicks);
            robot.backLeft.setTargetPosition(-distanceTicks);
            robot.backRight.setTargetPosition(distanceTicks);

        } else {
            robot.frontLeft.setTargetPosition(distanceTicks);
            robot.frontRight.setTargetPosition(distanceTicks);
            robot.backLeft.setTargetPosition(distanceTicks);
            robot.backRight.setTargetPosition(distanceTicks);
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

    @Override
    public void telemetry() {
        engine.telemetry.addLine();
        engine.telemetry.addData("Strafing", strafe);
        engine.telemetry.addData("lerp MM UP", lerpMM_UP);
        engine.telemetry.addData("lerp MM DOWN", lerpMM_DOWN);
        engine.telemetry.addData("Distance MM", distanceMM);
        engine.telemetry.addData("Distance Travelled MM", robot.distanceMM(robot.frontLeft));
        engine.telemetry.addData("Timeout MS", timeoutMS);
        progressBar(20, runTime() / timeoutMS);
    }

    private void tankMove(){
        double travelledDistance = Math.abs(robot.distanceMM(robot.frontLeft));
        double velocity = lerpVelocity(travelledDistance);

        double angleDiff = Utilities.angleDiff(initialHeadingDegrees, Utilities.facing(robot.imu));

        double leftVelocity = velocity;
        double rightVelocity = velocity;
        // use +10% of power at 7 degrees of error to correct angle
        double correctiveVelocity = Utilities.lerp(0.0, 1.0, angleDiff / 7.0) * (velocity * 0.1);
        if (angleDiff > -0.5) {
            leftVelocity += correctiveVelocity;
        } else if (angleDiff < 0.5) {
            rightVelocity += correctiveVelocity;
        }

        robot.frontLeft.setVelocity(leftVelocity);
        robot.frontRight.setVelocity(rightVelocity);
        robot.backLeft.setVelocity(leftVelocity);
        robot.backRight.setVelocity(leftVelocity);

        if (runTime() >= timeoutMS ||
                (atTargetPosition(robot.frontLeft) || atTargetPosition(robot.frontRight)) ||
                Math.abs(robot.distanceMM(robot.frontLeft)) >= Math.abs(distanceMM)) {
            robot.frontLeft.setVelocity(0);
            robot.frontRight.setVelocity(0);
            robot.backLeft.setVelocity(0);
            robot.backRight.setVelocity(0);

            this.finished();
        }
    }

    private void strafeMove() {
        double travelledDistance = Math.abs(robot.distanceMM(robot.frontLeft));
        double velocity = lerpVelocity(travelledDistance);

        double angleDiff = Utilities.angleDiff(initialHeadingDegrees, Utilities.facing(robot.imu));

        double frontVelocity = velocity;
        double backVelocity = velocity;
        // use +40% of power at 7 degrees of error to correct angle
        double correctiveVelocity = Utilities.lerp(0.0, 1.0, angleDiff / 7.0) * (velocity * 0.40);
        if (angleDiff > -0.5) {
            frontVelocity += correctiveVelocity;
        } else if (angleDiff < 0.5) {
            backVelocity += correctiveVelocity;
        }

        robot.frontLeft.setVelocity(frontVelocity);
        robot.frontRight.setVelocity(-frontVelocity);
        robot.backLeft.setVelocity(-backVelocity);
        robot.backRight.setVelocity(backVelocity);

        if (runTime() >= timeoutMS || (atTargetPosition(robot.frontLeft) || atTargetPosition(robot.backRight)) ||
            Math.abs(robot.distanceMM(robot.frontLeft)) >= Math.abs(distanceMM) || Math.abs(robot.distanceMM(robot.backRight)) >= Math.abs(distanceMM)) {
            robot.frontLeft.setVelocity(0);
            robot.frontRight.setVelocity(0);
            robot.backLeft.setVelocity(0);
            robot.backRight.setVelocity(0);

            this.finished();
        }
    }

    private double lerpVelocity(double travelledDistance) {
        double lerpVelocity = maxVelocityMM;

        // Ease power up
        if (travelledDistance < lerpMM_UP) { // Not using <= to prevent divide by zero
            lerpVelocity = Utilities.lerp(minVelocityMM, maxVelocityMM, Range.clip(travelledDistance / lerpMM_UP, 0.0, 1.0));
            // Cruising power
        } else if (travelledDistance < Math.abs(distanceMM) - lerpMM_DOWN) {
            lerpVelocity = maxVelocityMM;
            // Ease power down
        } else {
            lerpVelocity = Utilities.lerp(minVelocityMM, maxVelocityMM, Range.clip( (Math.abs(distanceMM) - travelledDistance) / lerpMM_DOWN, 0.0, 1.0));
        }

        return lerpVelocity;
    }

    private boolean atTargetPosition(DcMotorEx motor) {
        double travelledDistanceMM = robot.distanceMM(motor);

        return Utilities.isBetween(travelledDistanceMM, distanceMM - toleranceMM, distanceMM + toleranceMM);
    }
}
