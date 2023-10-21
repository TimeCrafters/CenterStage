package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class DepositorArmPosState extends CyberarmState {

    private final boolean stateDisabled;
    PrototypeRobot robot;

    public DepositorArmPosState(PrototypeRobot robot, String groupName, String actionName) {
        this.robot = robot;
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
    }

    @Override
    public void start() {
        //add variables that need to be reinitillized
    }

    @Override
    public void exec() {
        if (robot.armPosition == 0) { // transfer mode
            if (robot.oldArmPosititon == 0){
                setHasFinished(true);
            } else if (robot.oldArmPosititon == 1){
                robot.depositorElbow.setPosition(robot.ELBOW_COLLECT);
            } else if (robot.oldArmPosititon == 2){

            }
//        } else if (robot.armPosition == 1) { // drive mode
//            if (robot.oldArmPosititon == 1){
//                setHasFinished(true);
//            } else if (robot.oldArmPosititon == 0){
//
//            } else if (robot.oldArmPosititon == 2){
//
//            }
//        } else if (robot.armPosition == 2) { // deposit mode
//            if (robot.oldArmPosititon == 2){
//                setHasFinished(true);
//            } else if (robot.oldArmPosititon == 0){
//
//            } else if (robot.oldArmPosititon == 1){
//
//            }
        }
    }
}
