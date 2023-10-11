package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.CenterStage.Common.ProtoBotSodi;


public class ProtoBotStateSodi extends CyberarmState {
    ProtoBotSodi robot;
    public ProtoBotStateSodi(ProtoBotSodi robot) {
        this.robot = robot;
    }
    public void telemetry() {

    }
    @Override
    public void start() {

    //Motors
    robot.flDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.frDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.blDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.brDrive.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.bloodWorm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    //Servos
    robot.jaw.setDirection(Servo.Direction.FORWARD);
    }
    @Override
    public void exec() {

    }
}
