package org.timecrafters.CenterStage.States;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class PrototypeRobotDrivetrainState extends CyberarmState {
    private PrototypeRobot robot;

    public PrototypeRobotDrivetrainState(PrototypeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void exec() {
        robot.driveTrainTeleOp();

        if (engine.gamepad1.y){
            robot.depositorFlip.setPosition(0.75);
        } else if (engine.gamepad1.a){
            robot.depositorFlip.setPosition(0.05);
        }

        // depositor
        if (engine.gamepad1.b){
            robot.depositor.setPosition(0.8);
        } else if (engine.gamepad1.x){
            robot.depositor.setPosition(0.2);
        }
    }
}
