package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

public class RevHubTestSuiteAnalogTestsState extends RevTestSuiteTestState {
    private final ArrayList<AnalogInput> sensors;
    private int sensor_index = 0;
    public RevHubTestSuiteAnalogTestsState(RevHubTestSuiteRobot robot) {
        super(robot);

        sensors = robot.testingControlHub ? robot.controlHubAnalogSensors : robot.expansionHubAnalogSensors;
    }

    @Override
    public void exec() {
        super.exec();

        if (testComplete)
            return;

        if (sensor_index >= sensors.size()) {
            nextStage();
            report("-"); // Add newline
        }

        if (stage.ordinal() > STAGE.ANALOG_SENSOR.ordinal()) {
            testComplete = true;
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addLine("ANALOG SENSOR PORT TESTING");
        engine.telemetry.addLine();

        if (sensor_index < 4) {
            AnalogInput sensor = sensors.get(sensor_index);

            engine.telemetry.addData("Analog Sensor", "Port: %d, V: %.2f/%.2f (%.2f%%)", sensor_index, sensor.getVoltage(), sensor.getMaxVoltage(), (sensor.getVoltage() / sensor.getMaxVoltage()) * 100.0);
            engine.telemetry.addLine();

            engine.telemetry.addLine("MANUAL TEST");
            engine.telemetry.addLine("PRESS `A` if Analog Sensor " + sensor_index + " value is OKAY.");
            engine.telemetry.addLine();
            engine.telemetry.addLine("PRESS `Y` if Analog Sensor " + sensor_index + " value is NOT OKAY.");
            engine.telemetry.addLine();
        }

        super.telemetry();
    }

    @Override
    public void buttonUp(Gamepad gamepad, String button) {
        if (gamepad == engine.gamepad1) {
            if (button.equals("a")) {
                report("PASSED: Analog Sensor " + sensor_index + " OKAY");

                sensor_index++;
            } else if (button.equals("y")) {
                report("FAILED: Analog Sensor " + sensor_index + " NOT OKAY");

                sensor_index++;
            }
        }
    }
}
