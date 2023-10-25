package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.ProtoBotSodi;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class ArmStateSodi extends CyberarmState {

    private final boolean stateDisabled;
    ProtoBotSodi robot;
    private int testSequence;

    public ArmStateSodi(ProtoBotSodi robot, String groupName, String actionName) {
        this.robot = robot;
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
    }

    @Override
    public void start() {
        //add variables that need to be reinitialized
        //NOT REINITILLIZED >:(
    }

    @Override
    public void exec() {

        if (robot.liftMotor.motor.getCurrentPosition() < 0) {
            robot.liftMotor.motor.setPower(0);
        }
        if (robot.liftMotor.motor.getCurrentPosition() >= 0 && robot.liftMotor.motor.getCurrentPosition() <= 49) {

        }
        if (robot.liftMotor.motor.getCurrentPosition() >= 50 && robot.liftMotor.motor.getCurrentPosition() <= 250) {

        }


    }
}
