package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

public class RevHubTestSuiteServoTestsState extends RevTestSuiteTestState {
    private final int STAGE_SERVO_ROTATING = 0;
    private int servo_index = 0;
    private final ArrayList<ServoEx> servos;
    private double lastMonitorTime;
    private double automaticInterval = 1500.0; // milliseconds
    private boolean setInitialValue = true;
    public RevHubTestSuiteServoTestsState(RevHubTestSuiteRobot robot) {
        super(robot);

        servos = robot.testingControlHub ? robot.controlHubServos : robot.expansionHubServos;
        lastMonitorTime = runTime();
    }

    @Override
    public void exec() {
        super.exec();

        test_servos();

        if (stage > STAGE_SERVO_ROTATING) {
            testComplete = true;
        }
    }

    private void test_servos() {
        if (servo_index >= servos.size()) {
            stage += 1;
            return;
        }

        ServoEx servo = servos.get(servo_index);

        if (setInitialValue) {
            setInitialValue = false;
            lastMonitorTime = runTime();

            servo.setPosition(0);
        }

        if (runTime() - lastMonitorTime >= automaticInterval) {
            lastMonitorTime = runTime();

            servo.setPosition(servo.getPosition() == 0 ? 1 : 0);
        }
    }

    public void buttonUp(Gamepad gamepad, String button) {
        if (stage != STAGE_SERVO_ROTATING) {
            return;
        }

        if (engine.gamepad1 == gamepad) {
            if (button.equals("a")) {
                report("PASSED: Servo " + servo_index + " ROTATES");

                setInitialValue = true;
                servo_index++;
            } else if (button.equals("y")) {
                report("FAILED: Servo " + servo_index + " does NOT ROTATE");

                setInitialValue = true;
                servo_index++;
            }
        }
    }

    public void telemetry() {
        if (stage == STAGE_SERVO_ROTATING) {
            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Servo " + servo_index + " is ROTATING.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Servo " + servo_index + " is NOT ROTATING.");
            engine.telemetry.addLine();
        }

        super.telemetry();
    }
}
