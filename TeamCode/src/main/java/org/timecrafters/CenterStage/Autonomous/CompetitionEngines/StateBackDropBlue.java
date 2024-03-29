package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.CameraVisionState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawFingerState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesTask;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.OdometryLocalizerTask;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "State BackDrop blue", preselectTeleOp = "Competition V1 TeleOp")

public class StateBackDropBlue extends CyberarmEngine {

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
        this.robot = new CompetitionRobotV1("State BackDrop blue");
        addTask(new DriveToCoordinatesTask(robot));
        addTask(new OdometryLocalizerTask(robot));

        this.robot.setup();

        addState(new CameraVisionState(robot));

        addState(new ClawArmState(robot,"State BackDrop blue", "1-00-0"));

        // drive to the left, center, or right spike mark
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "2-03-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "2-03-1"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "2-03-2"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "2-02-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "2-01-0"));

        // bring arm to hover
        addState(new ClawArmState(robot,"State BackDrop blue", "3-00-0"));

        //open claw
        addState(new ClawFingerState(robot,"State BackDrop blue", "4-00-0"));

        addState(new ClawArmState(robot,"State BackDrop blue", "5-00-0"));

        // drive towards backboard
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-01-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-01-1"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-02-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-02-1"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-03-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-03-1"));

        // pause
        addState(new ClawArmState(robot,"State BackDrop blue", "6-00-0"));

        // drive into board
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-01-2"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-02-2"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "6-03-2"));

        // pause
        addState(new ClawArmState(robot,"State BackDrop blue", "6-00-1"));

        //open right close left
        addState(new ClawFingerState(robot,"State BackDrop blue", "7-00-0"));

        // bring arm up
        addState(new ClawArmState(robot,"State BackDrop blue", "8-00-0"));

        // drivw to park
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "9-01-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "9-02-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "9-03-0"));
        addState(new DriveToCoordinatesState(robot,"State BackDrop blue", "9-03-1"));

        addState(new ClawArmState(robot,"State BackDrop blue", "9-00-0"));



    // 65, on the left, 235 on the right






    }

}
