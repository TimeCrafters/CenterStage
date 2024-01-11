package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.CameraVisionState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawFingerState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "BURNSVILLE blue audience")

public class Competition2BlueBackStage extends CyberarmEngine {

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
        this.robot = new CompetitionRobotV1("burnsville audience blue");
        this.robot.setup();
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "0-00-0"));

        addState(new ClawArmState(robot,"burnsville audience blue", "0-01-0"));

//        addState(new CameraVisionState(robot));

        addState(new ClawArmState(robot,"burnsville audience blue", "0-01-1"));

        // drive to the left, center, or right spike mark
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-02-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-02-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-02-0"));

        // place pixel
        addState(new ClawFingerState(robot,"burnsville audience blue", "0-02-1"));

        // drive to search pos
        addState(new ClawArmState(robot,"burnsville audience blue", "0-02-2"));

        // close right finger
        addState(new ClawFingerState(robot,"burnsville audience blue", "0-02-3"));

        // drive back and away from the spike mark (x,y) (1050, 1000) H = 0
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-03-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-03-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-03-0"));


        // drive to the middle truss (right version) (1130,980) H = -90
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-04-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-04-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-04-0"));

        // drive to hover mode to clear the middle truss
        addState(new ClawArmState(robot,"burnsville audience blue", "0-04-1"));


        // drive under the middle truss across the field (right version) (1170,1080) H = -90
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-05-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-05-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-05-0"));

        // drive to deposit mode to place on back drop
        addState(new ClawArmState(robot,"burnsville audience blue", "0-05-1"));

        // drive to back drop
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-06-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-06-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-06-0"));

        // open claw to deposit gold
        addState(new ClawFingerState(robot,"burnsville audience blue", "0-06-1"));

        // drive back from backdrop
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-07-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-07-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-07-0"));

        // close left claw
        addState(new ClawFingerState(robot,"burnsville audience blue", "0-07-1"));


        // drive to parking spot
        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "3-08-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "2-08-0"));
//        addState(new DriveToCoordinatesState(robot,"burnsville audience blue", "1-08-0"));

        // arm to collect pos
        addState(new ClawArmState(robot,"burnsville audience blue", "0-08-1"));

    }

}
