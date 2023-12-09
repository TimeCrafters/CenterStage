package org.timecrafters.CenterStage.Autonomous.CompetitionEngines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.timecrafters.CenterStage.Autonomous.CompetitionStates.ClawArmState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DistanceCheckState;
import org.timecrafters.CenterStage.Autonomous.CompetitionStates.DriveToCoordinatesState;
import org.timecrafters.CenterStage.Common.CompetitionRobotV1;

import dev.cyberarm.engine.V2.CyberarmEngine;

@Autonomous(name = "Competition Red Close")

public class CompetitionRedClose extends CyberarmEngine {

    CompetitionRobotV1 robot;


    @Override
    public void setup() {
        this.robot = new CompetitionRobotV1("Competition Red Close");
        this.robot.setup();

        addState(new DriveToCoordinatesState(robot,"Competition Red Close", "00-1"));
        addState(new DistanceCheckState(robot,"Competition Red Close", "00-2"));

        addState(new ClawArmState(robot,"Competition Red Close", "01-0"));
    }

}
