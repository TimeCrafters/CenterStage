package org.timecrafters.CenterStage.Autonomous.CompetitionStates;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmState;

@Config
public class DriveToCoordinatesTask extends CyberarmState {

    CompetitionRobotV1 robot;
    public DriveToCoordinatesTask(CompetitionRobotV1 robot) {this.robot = robot;}
    @Override
    public void exec() {
        robot.XDrivePowerModifier();
        robot.YDrivePowerModifier();
        robot.HDrivePowerModifier();
        robot.DriveToCoordinates();
    }
}
