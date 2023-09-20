package org.timecrafters.diagnostics.rev_hub_test_suite.states;

import android.util.Log;

import org.timecrafters.diagnostics.rev_hub_test_suite.RevHubTestSuiteRobot;

import java.util.ArrayList;

import dev.cyberarm.engine.V2.CyberarmState;

public class RevTestSuiteTestState extends CyberarmState {
    final String TAG = "RevTestSuite|State";
    RevHubTestSuiteRobot robot;
    protected int stage = 0;
    protected ArrayList<String> reports;
    protected boolean testComplete = false;
    public RevTestSuiteTestState(RevHubTestSuiteRobot robot) {
        super();

        this.robot = robot;
    }

    @Override
    public void exec() {
        if (testComplete && engine.gamepad1.guide) {
            setHasFinished(true);
        }
    }

    public void report(String reason) {
        reports.add(reason);
    }

    @Override
    public void telemetry() {
        if (testComplete) {
            engine.telemetry.addLine("TESTING");
        } else {
            engine.telemetry.addLine("PRESS `GUIDE` TO CONTINUE");
        }
        engine.telemetry.addLine();
        engine.telemetry.addData("STAGE", stage);
        engine.telemetry.addLine();
        engine.telemetry.addLine("REPORTS");
        for (String report : reports) {
            engine.telemetry.addLine(report);
        }
    }
}
