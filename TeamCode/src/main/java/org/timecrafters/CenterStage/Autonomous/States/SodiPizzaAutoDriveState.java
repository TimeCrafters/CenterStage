package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;

    public SodiPizzaAutoDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void exec() {
        robot.flDrive.setTargetPosition(2800);
        if (robot.flDrive.getCurrentPosition() < robot.flDrive.getTargetPosition()) {
            robot.flDrive.setPower(0.75);
            robot.frDrive.setPower(0.75);
            robot.blDrive.setPower(0.75);
            robot.brDrive.setPower(0.75);
        } else {
            robot.flDrive.setPower(0);
            robot.frDrive.setPower(0);
            robot.blDrive.setPower(0);
            robot.brDrive.setPower(0);
        }
            if (robot.flDrive.getPower() == 0 && robot.flDrive.getCurrentPosition() >= robot.flDrive.getTargetPosition()) {
                setHasFinished(true);
            }
        }
    }
