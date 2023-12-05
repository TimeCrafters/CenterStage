package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;
import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class DriveToCoordinatesState extends CyberarmState {

    CompetitionRobotV1 robot;
    public double xTarget;
    public double yTarget;
    public double hTarget;
    public boolean posAchieved = false;

    public DriveToCoordinatesState(CompetitionRobotV1 robot/*, String groupName, String actionName*/) {
        this.robot = robot;
//        this.xTarget = robot.configuration.variable(groupName, actionName, "xTarget").value();
//        this.yTarget = robot.configuration.variable(groupName, actionName, "yTarget").value();
//        this.hTarget = robot.configuration.variable(groupName, actionName, "hTarget").value();
    }

    @Override
    public void exec() {
        robot.hTarget = hTarget;
        robot.yTarget = yTarget;
        robot.xTarget = xTarget;

        if (posAchieved){
//            setHasFinished(true);
        } else {
            robot.OdometryLocalizer();
            if (((robot.positionX > xTarget - 1) && (robot.positionX < xTarget + 1)) &&
            ((robot.positionH > hTarget - 1) && (robot.positionH < hTarget + 1)) &&
            ((robot.positionY > yTarget - 1) && (robot.positionY < yTarget + 1))){
                posAchieved = true;
            }
        }
    }


    @Override
    public void telemetry() {
        engine.telemetry.addData("x pos", robot.positionX);
        engine.telemetry.addData("y pos", robot.positionY);
        engine.telemetry.addData("h pos odo", robot.positionH);
        engine.telemetry.addData("aux encoder", robot.odometerA.getCurrentPosition());
        engine.telemetry.addData("left encoder", robot.odometerL.getCurrentPosition());
        engine.telemetry.addData("right encoder", robot.odometerR.getCurrentPosition());
        engine.telemetry.addData("h pos imu", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
