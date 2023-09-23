package org.timecrafters.diagnostics.rev_hub_test_suite;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteAnalogTestsState;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteDigitalTestsState;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteHubSelectionState;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteI2CTestsState;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteMotorTestsState;
import org.timecrafters.diagnostics.rev_hub_test_suite.states.RevHubTestSuiteServoTestsState;

import dev.cyberarm.engine.V2.CyberarmEngine;

//@Disabled
@TeleOp(name = "Rev Hub Test Suite", group = "diagnostics")
public class RevHubTestSuiteEngine extends CyberarmEngine {
    final String TAG = "RevTestSuite|Engine";
    RevHubTestSuiteRobot robot;

    @Override
    public void setup() {
        this.robot = new RevHubTestSuiteRobot();

        addState(new RevHubTestSuiteHubSelectionState(robot));

        addState(new RevHubTestSuiteMotorTestsState(robot));
        addState(new RevHubTestSuiteServoTestsState(robot));
        addState(new RevHubTestSuiteAnalogTestsState(robot));
        addState(new RevHubTestSuiteDigitalTestsState(robot));
        addState(new RevHubTestSuiteI2CTestsState(robot));
    }

    @Override
    public void loop() {
        robot.clearStaleData();

        super.loop();
    }
}
