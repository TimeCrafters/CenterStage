package org.timecrafters.CenterStage.TeleOp.States;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class ArmPosTest extends CyberarmState {

    private float armPos;
    private long lastMeasuredTime;
    private long waitTime;
    PrototypeRobot robot;
    public ArmPosTest(PrototypeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        armPos = 0F;
        lastMeasuredTime = System.currentTimeMillis();
        waitTime = 500;

        robot.depositorShoulder.setPosition(0);
        robot.depositorElbow.setPosition(0);
    }

    @Override
    public void exec(){

        if (engine.gamepad1.a){
            robot.depositorShoulder.setPosition(0);
            robot.depositorElbow.setPosition(0);
        } else if (engine.gamepad1.y){

            robot.depositorShoulder.setPosition(0.9);
            robot.depositorElbow.setPosition(0.22);
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("arm pos current", armPos);
    }
}
