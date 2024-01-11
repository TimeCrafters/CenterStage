package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;
@Config
public class DriveToCoordinatesState extends CyberarmState {

    CompetitionRobotV1 robot;
    public double xTarget;
    public double yTarget;
    public double hTarget;
    public boolean posAchieved = false;
    public boolean armDrive;
    public int objectPos;
    public boolean posSpecific;
    public double maxXPower;
    public double maxYPower;
    private String actionName;

    public DriveToCoordinatesState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.actionName = actionName;

        this.robot = robot;
        this.xTarget = robot.configuration.variable(groupName, actionName, "xTarget").value();
        this.yTarget = robot.configuration.variable(groupName, actionName, "yTarget").value();
        this.hTarget = robot.configuration.variable(groupName, actionName, "hTarget").value();
        this.maxXPower = robot.configuration.variable(groupName, actionName, "maxXPower").value();
        this.maxYPower = robot.configuration.variable(groupName, actionName, "maxYPower").value();
        this.armDrive = robot.configuration.variable(groupName, actionName, "armDrive").value();
        this.objectPos = robot.configuration.variable(groupName, actionName, "objectPos").value();
        this.posSpecific = robot.configuration.variable(groupName, actionName, "posSpecific").value();
    }

    @Override
    public void start() {
        super.start();
        robot.hTarget = hTarget;
        robot.yTarget = yTarget;
        robot.xTarget = xTarget;
        robot.yMaxPower = maxYPower;
        robot.xMaxPower = maxXPower;

        Log.d("TTT?", ""  + actionName + " CURRENT POSITION: x: " + robot.positionX + " Y: " + robot.positionY + "h: " + robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        Log.d("TTT?", ""  + actionName + " TARGET POSITION: x: " + robot.xTarget + " Y: " + robot.yTarget + "h: " + robot.hTarget);

    }

    @Override
    public void exec() {
        if (posSpecific) {
            if (objectPos != robot.objectPos) {
                // enter the ending loop
                setHasFinished(true);
            }
        }

        if (armDrive) {
            robot.clawArmControl();
        }

        robot.OdometryLocalizer();
        robot.XDrivePowerModifier();
        robot.YDrivePowerModifier();
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();

        if (Math.abs(robot.positionX - robot.xTarget) < 2
                && Math.abs(robot.positionY - robot.yTarget) < 2){
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
        engine.telemetry.addData("input y pidPower", robot.pidX);
        engine.telemetry.addData("input x pidPower", robot.pidY);
        engine.telemetry.addData("raw x pid", robot.XPIDControl(robot.xTarget, robot.positionX));
        engine.telemetry.addData("raw y pid", robot.YPIDControl(robot.yTarget, robot.positionY));
        engine.telemetry.addData("global object position", robot.objectPos);
        engine.telemetry.addData("local object position", objectPos);

    }
}
