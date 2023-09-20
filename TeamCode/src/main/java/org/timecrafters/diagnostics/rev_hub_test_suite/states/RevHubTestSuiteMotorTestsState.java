package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

public class RevHubTestSuiteMotorTestsState extends RevTestSuiteTestState {
    private final int STAGE_ENCODER_STEADY = 0; // Encoder value doesn't change by more than 2 while motor is off.
    private final int STAGE_ENCODER_CHANGES = 1; // Encoder value is changes when motor is running
    private final int STAGE_MOTOR_BRAKING = 2; // Motor brake mode works (manual)

    private int motor_index = 0;

    private final ArrayList<MotorEx> motors;
    private double lastMonitorTime;
    private double automaticInterval = 500.0; // milliseconds

    private int initialValue, lastValue;
    private boolean setInitialValue = true;
    public RevHubTestSuiteMotorTestsState(RevHubTestSuiteRobot robot) {
        super(robot);

        motors = robot.testingControlHub ? robot.controlHubMotors : robot.expansionHubMotors;
        lastMonitorTime = runTime();
    }

    @Override
    public void exec() {
        super.exec();

        switch (stage) {
            case STAGE_ENCODER_STEADY: {
                test_encoder_steady();
            }
            case STAGE_ENCODER_CHANGES: {
                test_encoder_changes();
            }
            case STAGE_MOTOR_BRAKING: {
                test_motor_braking();
            }
        }

        if (stage > STAGE_MOTOR_BRAKING) {
            testComplete = true;
        }
    }

    protected void test_encoder_steady() {
        if (motor_index >= motors.size()) {
            stage += 1;
            motor_index = 0;
            setInitialValue = true;
            return;
        }

        MotorEx motor = motors.get(motor_index);

        if (setInitialValue) {
            setInitialValue = false;
            initialValue = motor.getCurrentPosition();
            lastMonitorTime = runTime();
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastValue = Math.abs(initialValue - motor.getCurrentPosition());

            if (lastValue <= 2) {
                report("PASSED: Motor Encoder " + motor_index + " STEADY");
                this.motor_index++;
            } else {
                report("FAILED: Motor Encoder " + motor_index + " STEADY (" + lastValue + " > 2)");
            }
        }
    }

    protected void test_encoder_changes() {
        if (motor_index >= motors.size()) {
            stage += 1;
            motor_index = 0;
            setInitialValue = true;
            return;
        }

        MotorEx motor = motors.get(motor_index);

        if (setInitialValue) {
            setInitialValue = false;
            initialValue = motor.getCurrentPosition();
            lastMonitorTime = runTime();

            motor.setVelocity(0.25);
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastValue = Math.abs(initialValue - motor.getCurrentPosition());

            if (lastValue >= 100) {
                motor.stopMotor();
                report("PASSED: Motor Encoder `" + motor_index + "` CHANGE");
                this.motor_index++;
            } else {
                report("FAILED: Motor Encoder `" + motor_index + "` CHANGE (" + lastValue + " < 100)");
            }
        }
    }

    protected void test_motor_braking() {
        if (motor_index >= motors.size()) {
            stage += 1;
            motor_index = 0;
            setInitialValue = true;
            return;
        }

        MotorEx motor = motors.get(motor_index);

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (stage != STAGE_MOTOR_BRAKING) {
            return;
        }

        if (engine.gamepad1 == gamepad) {
            if (button.equals("a")) {
                report("PASSED: Motor " + motor_index + " BRAKES");

                try {
                    MotorEx motor = motors.get(motor_index);
                    motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
                } catch (IndexOutOfBoundsException e) { /* no op */ }

                motor_index++;
            } else if (button.equals("y")) {
                report("FAILED: Motor " + motor_index + " does NOT BRAKE");

                try {
                    MotorEx motor = motors.get(motor_index);
                    motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
                } catch (IndexOutOfBoundsException e) { /* no op */ }

                motor_index++;
            }
        }
    }

    @Override
    public void telemetry() {
        if (stage == STAGE_MOTOR_BRAKING) {
            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Motor " + motor_index + " is BRAKING.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Motor " + motor_index + " is NOT BRAKING.");
            engine.telemetry.addLine();
        }

        super.telemetry();
    }
}
