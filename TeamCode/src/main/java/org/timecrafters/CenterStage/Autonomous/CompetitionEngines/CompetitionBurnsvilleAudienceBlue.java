package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmControlTask;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawFingerState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesTask;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.OdometryLocalizerTask;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Burnsville blue audience")

public class CompetitionBurnsvilleAudienceBlue extends CyberarmEngine {

    CompetitionRobotV1 robot;


    @Override
    public void init() {
        super.init();
        robot.clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.clawArm.setTargetPosition(0);
        robot.clawArm.setPower(0);
        robot.clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.imu.resetYaw();
        robot.leftClaw.setPosition(0.25);
        robot.rightClaw.setPosition(0.6);
    }

    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Burnsville audience blue");
        addTask(new DriveToCoordinatesTask(robot));
        addTask(new OdometryLocalizerTask(robot));
//        addTask(new ClawArmControlTask(robot));

        this.robot.setup();
        addState(new ClawArmState(robot,"Burnsville audience blue", "0-01-0"));

//        addState(new CameraVisionState(robot));

        addState(new ClawArmState(robot,"Burnsville audience blue", "0-01-1"));

        // drive to the left, center, or right spike mark
//        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "3-02-0"));
//        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "2-02-0"));
        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "1-02-0"));

        // place pixel
        addState(new ClawFingerState(robot,"Burnsville audience blue", "0-02-1"));

        // drive to search pos
        addState(new ClawArmState(robot,"Burnsville audience blue", "0-02-2"));

        // close right finger
        addState(new ClawFingerState(robot,"Burnsville audience blue", "0-02-3"));

//        // drive back and away from the spike mark (x,y) (1050, 1000) H = 0
////        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "3-03-0"));
////        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "2-03-0"));
//        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "1-03-0"));
//
//
//        // drive to the middle truss (right version) (1130,980) H = -90
////        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "3-04-0"));
////        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "2-04-0"));
//        addState(new DriveToCoordinatesState(robot,"Burnsville audience blue", "1-04-0"));
//
//        // drive to hover mode to clear the middle truss
//        addState(new ClawArmState(robot,"Burnsville audience blue", "0-04-1"));
//
    }

}
