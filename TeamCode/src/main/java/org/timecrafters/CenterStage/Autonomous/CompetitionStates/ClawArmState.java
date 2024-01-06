package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class ClawArmState extends CyberarmState {

    CompetitionRobotV1 robot;
    public String armPos;
    public long wait;
    public long initTime;
    public ClawArmState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
        this.armPos = robot.configuration.variable(groupName, actionName, "armPos").value();
        this.wait = robot.configuration.variable(groupName, actionName, "wait").value();
    }

    @Override
    public void start() {
        super.start();
        initTime = System.currentTimeMillis();
    }

    @Override
    public void exec() {
        // odometry driving ALWAYS
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();

        // driving arm to pos
        robot.armPos = armPos;
        robot.clawArmControl();

        if ((robot.clawArm.getCurrentPosition() > robot.target - 20 || robot.clawArm.getCurrentPosition() < robot.target + 20) && wait < System.currentTimeMillis() - initTime) {
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
