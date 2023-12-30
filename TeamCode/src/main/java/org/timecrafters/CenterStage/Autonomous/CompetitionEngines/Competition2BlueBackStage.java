package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.CameraVisionState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawFingerState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "BURNSVILLE blue backdrop")

public class Competition2BlueBackStage extends CyberarmEngine {

    CompetitionRobotV1 robot;


    @Override
    public void init() {
        super.init();
        robot.clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.imu.resetYaw();
        robot.leftClaw.setPosition(0.25);
        robot.rightClaw.setPosition(0.6);
    }

    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Burnsville backdrop blue");
        this.robot.setup();
        addState(new DriveToCoordinatesState(robot,"burnsville backdrop blue", "0-00-0"));
        addState(new ClawArmState(robot,"burnsville backdrop blue", "0-01-0"));
        addState(new CameraVisionState(robot));
        addState(new ClawArmState(robot,"burnsville backdrop blue", "0-02-0"));

        addState(new DriveToCoordinatesState(robot,"burnsville backdrop blue", "3-03-0"));















    }

}
