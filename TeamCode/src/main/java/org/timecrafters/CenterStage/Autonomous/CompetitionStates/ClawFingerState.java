package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class ClawFingerState extends CyberarmState {

    CompetitionRobotV1 robot;
    public boolean leftOpen;
    public boolean rightOpen;
    public long waitTime;
    public boolean armDrive;
    public long initTime;



    public ClawFingerState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
        this.leftOpen = robot.configuration.variable(groupName, actionName, "leftOpen").value();
        this.rightOpen = robot.configuration.variable(groupName, actionName, "rightOpen").value();
        this.waitTime = robot.configuration.variable(groupName, actionName, "waitTime").value();
        this.armDrive = robot.configuration.variable(groupName, actionName, "armDrive").value();


    }

    @Override
    public void start() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void exec() {
        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();
        if (armDrive) {
            robot.clawArmControl();
        }

        if (!leftOpen){
            robot.leftClaw.setPosition(0.25);
        } else {
            robot.leftClaw.setPosition(0.6);
        }

        if (!rightOpen){
            robot.rightClaw.setPosition(0.6);
        } else {
            robot.rightClaw.setPosition(0.25);
        }

        if (runTime() > waitTime){
            setHasFinished(true);
        }




    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("x pos", robot.positionX);
        engine.telemetry.addData("y pos", robot.positionY);
        engine.telemetry.addData("h pos odo", Math.toDegrees(robot.positionH));
        engine.telemetry.addData("aux encoder", robot.currentAuxPosition);
        engine.telemetry.addData("left encoder", robot.currentLeftPosition);
        engine.telemetry.addData("right encoder", robot.currentRightPosition);
        engine.telemetry.addData("h pos imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        engine.telemetry.addData("front right power", robot.frontRightPower);
        engine.telemetry.addData("front left power", robot.frontLeftPower);
        engine.telemetry.addData("back right power", robot.backRightPower);
        engine.telemetry.addData("back left power", robot.backLeftPower);
        engine.telemetry.addData("arm pos", robot.clawArm.getCurrentPosition());

    }
}
