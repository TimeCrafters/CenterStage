package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import dev.cyberarm.engine.V2.CyberarmState;
import org.timecrafters.CenterStage.Common.ProtoBotSodi;


public class ProtoBotStateSodi extends CyberarmState {
    ProtoBotSodi robot;
    private long lastTimeChecked;
    private int testSequence;
    private int targetPos;
    private int currentPos;
    private int totalDist;

    public ProtoBotStateSodi(ProtoBotSodi robot) {
        this.robot = robot;
    }
    public void telemetry() {
    engine.telemetry.addData("Front Left Velocity", robot.flDrive.getVelocity());
    engine.telemetry.addData("Front Right Velocity", robot.frDrive.getVelocity());
    engine.telemetry.addData("Back Left Velocity", robot.blDrive.getVelocity());
    engine.telemetry.addData("Back Right Velocity", robot.brDrive.getVelocity());
    engine.telemetry.addData("Front Left Power", robot.flDrive.motor.getPower());
    engine.telemetry.addData("Front Right Power", robot.frDrive.motor.getPower());
    engine.telemetry.addData("Back Left Power", robot.blDrive.motor.getPower());
    engine.telemetry.addData("Back Right Power", robot.brDrive.motor.getPower());

    }

    @Override
    public void init() {
        robot.flDrive.motor.setPower(0);
        robot.frDrive.motor.setPower(0);
        robot.blDrive.motor.setPower(0);
        robot.brDrive.motor.setPower(0);
//        robot.liftMotor.motor.setPower(0);
//
//        robot.grabJaw.setPosition(0);
//        robot.grabElbow.setPosition(0);
//        robot.grabShoulder.setPosition(0);
//        robot.dropShoulder.setPosition(0);
//        robot.dropElbow.setPosition(0);
//        robot.dropJaw.setPosition(0);

        lastTimeChecked = System.currentTimeMillis();
        testSequence = 0;




    }

    @Override
    public void exec() {

        currentPos = robot.flDrive.motor.getCurrentPosition();

        if (testSequence < 1) {
            testSequence = 1;
        }

        switch (testSequence) {
            case 1:

                robot.flDrive.motor.setTargetPosition(500);
                robot.frDrive.motor.setTargetPosition(500);
                robot.blDrive.motor.setTargetPosition(500);
                robot.brDrive.motor.setTargetPosition(500);

                if (robot.flDrive.motor.getCurrentPosition() < robot.flDrive.motor.getTargetPosition() - 50) {
                    robot.flDrive.motor.setPower(0.5 * (robot.flDrive.motor.getTargetPosition() - robot.flDrive.motor.getCurrentPosition()));
                    robot.frDrive.motor.setPower(0.5 * (robot.frDrive.motor.getTargetPosition() - robot.frDrive.motor.getCurrentPosition()));
                    robot.blDrive.motor.setPower(0.5 * (robot.blDrive.motor.getTargetPosition() - robot.blDrive.motor.getCurrentPosition()));
                    robot.brDrive.motor.setPower(0.5 * (robot.brDrive.motor.getTargetPosition() - robot.brDrive.motor.getCurrentPosition()));
                }
                else if (robot.flDrive.motor.getCurrentPosition() < robot.flDrive.motor.getTargetPosition() + 50 ||
                robot.flDrive.motor.getCurrentPosition() > robot.flDrive.motor.getTargetPosition() - 50) {

                    robot.flDrive.motor.setPower(0);
                    robot.frDrive.motor.setPower(0);
                    robot.blDrive.motor.setPower(0);
                    robot.brDrive.motor.setPower(0);

//                    testSequence = 2;
                }

                break;
//            case 2:





        }

    }
}
