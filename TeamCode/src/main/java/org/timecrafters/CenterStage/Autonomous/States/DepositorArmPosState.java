package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.PrototypeRobot;
import org.timecrafters.CenterStage.Common.ServoTestRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class DepositorArmPosState extends CyberarmState {

    private final boolean stateDisabled;
    ServoTestRobot robot;
    private long startOfSequencerTime;
    private long totalWaitedTime;
    private long lastMeasuredTime;
    private boolean actionsFinished = false;

    private int armPosition;

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
        if (actionsFinished) {
            setHasFinished(true);
        } else {
            lastMeasuredTime = System.currentTimeMillis();

            if (armPosition == robot.oldArmPosititon){
                actionsFinished = true;
            } else if (robot.oldArmPosititon == 1){

                robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN);
                robot.ServoWaitTime(robot.COLLECTOR_SHOULDER_PASSIVE, robot.COLLECTOR_SHOULDER_IN);
                totalWaitedTime = robot.servoWaitTime;
                if (System.currentTimeMillis() - startOfSequencerTime >= robot.servoWaitTime){
                    robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                    robot.ServoWaitTime(robot.COLLECTOR_ELBOW_PASSIVE, robot.COLLECTOR_ELBOW_IN);
                    totalWaitedTime += robot.servoWaitTime;
                    if (System.currentTimeMillis() - lastMeasuredTime >= totalWaitedTime){
                        actionsFinished = true;
                    }
                }
            }

            // -----------------------------------------------------------------------------------------------------0, drive to transfer
            // if already at 0, actions have finished = true

            // else if at 1,
            // drive collector shoulder to position
            // do math and wait till wait time is met
            // once met, drive the elbow
            // set has finished = true

            // else if at 2,
            // drive collector shoulder to position
            // do math and wait till wait time is met
            // once met, drive the elbow
            // set has finished = true

            // else if at 3,
            // drive collector shoulder to position
            // do math and wait till wait time is met
            // once met, drive the elbow
            // set has finished = true

            // -----------------------------------------------------------------------------------------------------1, drive to driving mode



            // -----------------------------------------------------------------------------------------------------2, drive to collect



            // -----------------------------------------------------------------------------------------------------3, drive to deposit





        }
    }
}
