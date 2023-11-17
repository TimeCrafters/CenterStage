package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class AutoStateScrimmage extends CyberarmState {

    private int distanceDriven;
    private int targetDistance = 38000;
    private int stepsFinished = 0;
    private boolean firstDrivePos = false;
    private boolean secondDrivePos = false;
    private boolean armToFIrstPos = false;
    private boolean armToSecondPos = false;
    private boolean collect = false;
    PrototypeRobot robot;
    public AutoStateScrimmage(PrototypeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {

        robot.odometerR.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    @Override
    public void exec() {
        distanceDriven = robot.currentRightPosition;

        if (stepsFinished == 5) {
//            setHasFinished(true);
        }

        if (robot.odometerR.getCurrentPosition() >= targetDistance && firstDrivePos == false){
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
            stepsFinished += 1;
            firstDrivePos = true;
            robot.startOfSequencerTime = System.currentTimeMillis();
        } else if (robot.odometerR.getCurrentPosition() < targetDistance){
            robot.frontRight.setPower(0.5);
            robot.frontLeft.setPower(0.5);
            robot.backRight.setPower(0.5);
            robot.backLeft.setPower(0.5);
        }

        if (stepsFinished == 1 && firstDrivePos && armToFIrstPos == false){
            robot.armPosition = 2;
            robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_OUT); // drive the shoulder to the transfer position
            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
                robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_OUT);
                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 2000) {
                    robot.oldArmPosition = 2;
                    armToFIrstPos = true;
                }
            }
        }

        if (firstDrivePos && armToFIrstPos){
            robot.collector.setPosition(1F);
            collect = true;
        }

//        if (stepsFinished == 3 && firstDrivePos && armToFIrstPos && collect && secondDrivePos == false){
//            if (robot.odometerR.getCurrentPosition() <= targetDistance - 500){
//            robot.frontRight.setPower(0);
//            robot.frontLeft.setPower(0);
//            robot.backRight.setPower(0);
//            robot.backLeft.setPower(0);
//            stepsFinished += 1;
//            robot.startOfSequencerTime = System.currentTimeMillis();
//            secondDrivePos = true;
//            } else {
//            robot.frontRight.setPower(-0.3);
//            robot.frontLeft.setPower(-0.3);
//            robot.backRight.setPower(-0.3);
//            robot.backLeft.setPower(-0.3);
//            }
//        }
        if (firstDrivePos && armToFIrstPos && collect){
            robot.collectorShoulder.setPosition(robot.COLLECTOR_SHOULDER_IN); // drive the shoulder to the transfer position
            if (System.currentTimeMillis() - robot.startOfSequencerTime >= 750) { // wait to move till time is met
                robot.collectorElbow.setPosition(robot.COLLECTOR_ELBOW_IN);
                if (System.currentTimeMillis() - robot.startOfSequencerTime >= 1500) {
                    robot.oldArmPosition = 0;
                }
            }
        }
    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("current pos", robot.odometerR.getCurrentPosition());
        engine.telemetry.addData("time", System.currentTimeMillis() - robot.startOfSequencerTime);
    }
}

