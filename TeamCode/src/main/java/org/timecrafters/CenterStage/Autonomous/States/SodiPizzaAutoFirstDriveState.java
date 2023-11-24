package org.timecrafters.CenterStage.Autonomous.States;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoFirstDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;

    public SodiPizzaAutoFirstDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }

    @Override
    public void start() {

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Ticks?", robot.leftFront.getTargetPosition());
        engine.telemetry.addData("Current Ticks?", robot.leftFront.getCurrentPosition());
        engine.telemetry.addData("Ticks Needed?", robot.leftFront.getTargetPosition() -
        robot.leftFront.getCurrentPosition());
    }

    @Override
    public void exec() {

        if (robot.leftFront.getCurrentPosition() < 1000) {

            robot.leftFront.setTargetPosition(1000);
            robot.leftBack.setTargetPosition(1000);
            robot.rightFront.setTargetPosition(1000);
            robot.rightBack.setTargetPosition(1000);

            robot.leftFront.setPower(0.5);
            robot.rightFront.setPower(0.5);
            robot.leftBack.setPower(0.5);
            robot.rightBack.setPower(0.5);

        } else {
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            setHasFinished(true);
        }
    }
}
