package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.CenterStage.Common.ProtoBotSodi;


public class ProtoBotStateSodi extends CyberarmState {
    ProtoBotSodi robot;
    public ProtoBotStateSodi() {
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
    public void init() {
        robot.flDrive.motor.setPower(0);
        robot.frDrive.motor.setPower(0);
        robot.blDrive.motor.setPower(0);
        robot.brDrive.motor.setPower(0);
        robot.bloodWorm.motor.setPower(0);
    }

    @Override
    public void exec() {

        if (System.currentTimeMillis() >= 500 && System.currentTimeMillis() < 2500) {
            robot.flDrive.motor.setPower(0.5);
            robot.frDrive.motor.setPower(0.5);
            robot.blDrive.motor.setPower(0.5);
            robot.brDrive.motor.setPower(0.5);
            robot.bloodWorm.motor.setPower(0.5);
        } else if (System.currentTimeMillis() >= 2500 && System.currentTimeMillis() < 3500) {
            robot.flDrive.motor.setPower(-0.5);
            robot.frDrive.motor.setPower(-0.5);
            robot.blDrive.motor.setPower(-0.5);
            robot.brDrive.motor.setPower(-0.5);
            robot.bloodWorm.motor.setPower(-0.5);
        } else {
            robot.flDrive.motor.setPower(0);
            robot.frDrive.motor.setPower(0);
            robot.blDrive.motor.setPower(0);
            robot.brDrive.motor.setPower(0);
            robot.bloodWorm.motor.setPower(0);
        }
    }
}
