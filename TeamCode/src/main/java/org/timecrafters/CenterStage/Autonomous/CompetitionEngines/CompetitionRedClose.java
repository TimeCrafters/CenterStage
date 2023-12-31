package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawFingerState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Competition Red Close")

public class CompetitionRedClose extends CyberarmEngine {

    CompetitionRobotV1 robot;

    @Override
    public void init() {
        super.init();
        robot.clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.imu.resetYaw();
    }

    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Competition Red Close");
        this.robot.setup();
        this.robot.setup();
        addState(new ClawFingerState(robot,"Competition Red Close", "01-0-00"));
        // drive to and face towards the right scenario
        addState(new DriveToCoordinatesState(robot,"Competition Red Close", "02-0-01"));
        // check if its in the correct Position;
        addState(new ClawArmState(robot,"Competition Red Close", "03-0-02"));
        addState(new ClawFingerState(robot,"Competition Red Close", "04-0-04"));
        addState(new DriveToCoordinatesState(robot,"Competition Red Close", "05-0-01"));
        addState(new ClawArmState(robot,"Competition Red Close", "06-0-06"));













    }

}
