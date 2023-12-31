package org.timecrafters.CenterStage.TeleOp.States;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class ArmPosTest extends CyberarmState {

    private float armPosE;
    private float armPosS;
    private long lastMeasuredTime;
    private long waitTime;
    PrototypeRobot robot;
    public ArmPosTest(PrototypeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        armPosS = 0F;
        armPosE = 0F;
        lastMeasuredTime = System.currentTimeMillis();
        waitTime = 500;

        robot.depositorShoulder.setPosition(0);
        robot.depositorElbow.setPosition(0);
    }

    @Override
    public void exec(){
        robot.collectorShoulder.setPosition(armPosS);
        robot.collectorElbow.setPosition(armPosE);

//        if (engine.gamepad1.a){
//            robot.collectorShoulder.setPosition(0.75);
//        } else if (engine.gamepad1.y){
//            robot.collectorShoulder.setPosition(0.65);
//        }else if (engine.gamepad1.x){
//            robot.collectorShoulder.setPosition(0.4);
//        }

        if (engine.gamepad1.y && System.currentTimeMillis() - lastMeasuredTime > 500){
            lastMeasuredTime = System.currentTimeMillis();
            armPosS += 0.05;
        } else if (engine.gamepad1.a && System.currentTimeMillis() - lastMeasuredTime > 500){
            lastMeasuredTime = System.currentTimeMillis();
            armPosS -= 0.05;
        }

        if (engine.gamepad2.y && System.currentTimeMillis() - lastMeasuredTime > 500){
            lastMeasuredTime = System.currentTimeMillis();
            armPosE += 0.05;
        } else if (engine.gamepad2.a && System.currentTimeMillis() - lastMeasuredTime > 500){
            lastMeasuredTime = System.currentTimeMillis();
            armPosE -= 0.05;
        }
    }



    @Override
    public void telemetry() {
        engine.telemetry.addData("arm pos current Shoulder", armPosS);
        engine.telemetry.addData("arm pos current Elbow", armPosE);
    }
}
