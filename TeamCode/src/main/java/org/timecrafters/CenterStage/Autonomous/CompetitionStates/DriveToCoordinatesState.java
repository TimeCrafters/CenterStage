package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;
@Config
public class DriveToCoordinatesState extends CyberarmState {

    CompetitionRobotV1 robot;
    public double xTarget = 0;
    public double yTarget = 0;
    public double hTarget = 0;
    public boolean posAchieved = false;
    public boolean armDrive;
    public String objectPos;
//    public boolean posSpecific;

    public DriveToCoordinatesState(CompetitionRobotV1 robot, String groupName, String actionName) {
        this.robot = robot;
        this.xTarget = robot.configuration.variable(groupName, actionName, "xTarget").value();
        this.yTarget = robot.configuration.variable(groupName, actionName, "yTarget").value();
        this.hTarget = robot.configuration.variable(groupName, actionName, "hTarget").value();
        this.armDrive = robot.configuration.variable(groupName, actionName, "armDrive").value();
        this.objectPos = robot.configuration.variable(groupName, actionName, "objectPos").value();
//        this.posSpecific = robot.configuration.variable(groupName, actionName, "posSpecific").value();
    }

    @Override
    public void exec() {
        robot.hTarget = hTarget;
        robot.yTarget = yTarget;
        robot.xTarget = xTarget;
        robot.DriveToCoordinates();
        robot.OdometryLocalizer();
        if (armDrive) {
            robot.clawArmControl();
        }

        if (posAchieved){
            setHasFinished(true);
        } else {
//            if (objectPos.equals(robot.objectPos) || objectPos.equals("everything") ) {
                if (Math.abs(robot.backLeftPower) < 0.15 &&
                   Math.abs(robot.backRightPower) < 0.15 &&
                   Math.abs(robot.frontLeftPower) < 0.15 &&
                   Math.abs(robot.frontRightPower) < 0.15) {
                    posAchieved = true;
//                }
            }
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
        engine.telemetry.addData("global object position", robot.objectPos);
        engine.telemetry.addData("local object position", objectPos);

    }
}
