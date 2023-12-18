package org.timecrafters.CenterStage.Autonomous.Engines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.CameraVisionState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "camera test")

public class CameraTestEngine extends CyberarmEngine {

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
        this.robot = new CompetitionRobotV1("Competition camera test");
        this.robot.setup();
        addState(new CameraVisionState(robot));








    }

}
