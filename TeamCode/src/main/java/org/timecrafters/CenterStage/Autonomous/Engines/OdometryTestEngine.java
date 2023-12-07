package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "odo test and tune")

public class OdometryTestEngine extends CyberarmEngine {

    CompetitionRobotV1 robot;

    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Autonomous odometry test");
        this.robot.setup();

        addState(new DriveToCoordinatesState(robot/*,"odoTest", "00-1"*/));
    }
    @Override
    public void init() {

        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
