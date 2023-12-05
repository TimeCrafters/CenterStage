package org.timecrafters.CenterStage.Autonomous.SodiStates;

import org.timecrafters.CenterStage.Common.ServoTestRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class DepositorArmPosState extends CyberarmState {

    private final boolean stateDisabled;
    ServoTestRobot robot;
    private long startOfSequencerTime;
    private long totalWaitedTime = 0;
    private boolean actionsFinished = false;
    private boolean enteredLoop = false;

    private int armPosition = 0;
    private int oldArmPosition;

    public DepositorArmPosState(ServoTestRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
        armPosition = robot.configuration.variable(groupName, actionName, "armPosition").value();
    }

    @Override
    public void start() {
        startOfSequencerTime = System.currentTimeMillis();
    }

    @Override
    public void exec() {

        switch (armPosition) {
            case 0: // ----------------------------------------------------------------------------------------------- drive to transfer pos
                switch (oldArmPosition) {
                    case 0:
                        // transfer
                        break;
                    case 1:
                        // driving
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        robot.ServoWaitTime(robot.COLLECTOR_SHOULDER_PASSIVE, robot.COLLECTOR_SHOULDER_IN); // calculate time to move
                        totalWaitedTime = (long) robot.servoWaitTime;
                        if (System.currentTimeMillis() - startOfSequencerTime >= totalWaitedTime) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                            enteredLoop = true;
                            robot.ServoWaitTime(robot.COLLECTOR_ELBOW_PASSIVE, robot.COLLECTOR_ELBOW_IN); // calculate time to move
                            totalWaitedTime += (long) robot.servoWaitTime; // add the time to the total time to wait
                            if (System.currentTimeMillis() - startOfSequencerTime >= totalWaitedTime) {
                        setHasFinished(true);
                            }
                        }
                        break;
                    case 2:
                        // collect
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 3:
                        // deposit
                        robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 800) { // wait to move till time is met
                            robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1600) {
                                robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                                if (System.currentTimeMillis() - startOfSequencerTime >= 2300) { // wait to move till time is met
                                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                                    if (System.currentTimeMillis() - startOfSequencerTime >= 3100) {
                                        setHasFinished(true);
                                    }
                                }
                            }
                        }
                        break;
                }
                break;

            case 1:// ----------------------------------------------------------------------------------------------- drive to driving pos
                switch (oldArmPosition) {
                    case 0:
                        // transfer
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 1:
                        // drive pos
                        setHasFinished(true);
                        break;
                    case 2:
                        // collect
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 600) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 2100) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 3:
                        // deposit
                        robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 800) { // wait to move till time is met
                            robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1600) {
                                robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE); // drive the shoulder to the transfer position
                                if (System.currentTimeMillis() - startOfSequencerTime >= 2300) { // wait to move till time is met
                                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_PASSIVE);
                                    if (System.currentTimeMillis() - startOfSequencerTime >= 3100) {
                                        setHasFinished(true);
                                    }
                                }
                            }
                        }
                        break;
                }
                break;

            case 2:// ----------------------------------------------------------------------------------------------- drive to collect pos
                switch (oldArmPosition) {
                    case 0:
                        // transfer
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_OUT);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 1:
                        // driving
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_OUT);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 2:
                        // collect
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 3:
                        // deposit
                        robot.depositorShoulder.setPosition(robot.DEPOSITOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 100) { // wait to move till time is met
                            robot.depositorElbow.setPosition(robot.DEPOSITOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 100) {
                                if (System.currentTimeMillis() - startOfSequencerTime >= 100) { // wait to move till time is met
                                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT);
                                    if (System.currentTimeMillis() - startOfSequencerTime >= 100) {
                                        setHasFinished(true);
                                    }
                                }
                            }
                        }
                        break;
                }
                break;

            case 3:// ----------------------------------------------------------------------------------------------- drive to deposit pos
                switch (oldArmPosition) {
                    case 0:
                        // transfer
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 1:
                        // driving
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_ELBOW_PASSIVE); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 700) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_SHOULDER_PASSIVE);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 2000) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 2:
                        // collect
                        robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
                        if (System.currentTimeMillis() - startOfSequencerTime >= 750) { // wait to move till time is met
                            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                            if (System.currentTimeMillis() - startOfSequencerTime >= 1500) {
                                setHasFinished(true);
                            }
                        }
                        break;
                    case 3:
                        // deposit
                        break;
                }
                break;

        }
        }

    @Override
    public void telemetry(){
        engine.telemetry.addData("Current Time",System.currentTimeMillis()-startOfSequencerTime);
        engine.telemetry.addData("servo wait time",robot.servoWaitTime);
        engine.telemetry.addData("servo wait time (long)", (long)robot.servoWaitTime);
        engine.telemetry.addData("entered loop",enteredLoop);
        }
}
