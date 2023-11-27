package org.timecrafters.CenterStage.Autonomous.States;

import android.annotation.SuppressLint;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoFirstDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;
    private int targetPos = 2500;
    private double drivePower;

    public SodiPizzaAutoFirstDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }
    
    @SuppressLint("SuspiciousIndentation")

    @Override
    public void start() {

        lastMoveTime = System.currentTimeMillis();
        robot.readyToTurn = 0;

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
        // Move forward from 0 to targetPos
        if (robot.leftFront.getCurrentPosition() <= 10 && robot.leftFront.getCurrentPosition() >= -10) {

            robot.leftFront.setTargetPosition(targetPos);
            robot.leftBack.setTargetPosition(targetPos);
            robot.rightFront.setTargetPosition(targetPos);
            robot.rightBack.setTargetPosition(targetPos);

            drivePower = 0.5;

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

        }
        //Stop and finish set after return to 0
        else if (robot.leftFront.getCurrentPosition() >= targetPos - 10 && robot.leftFront.getCurrentPosition() <= targetPos + 10) {

            drivePower = 0;

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

            robot.readyToTurn = 1;

            setHasFinished(true);
        }

/*        //Stop moving and update lastMoveTime
        if (robot.leftFront.getCurrentPosition() >= targetPos && drivePower > 0) {

            lastHalf = true;

            drivePower = 0;

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

            lastMoveTime = System.currentTimeMillis();
        }

        //Move backwards from targetPos to 0
        if (robot.leftFront.getCurrentPosition() >= targetPos && drivePower == 0
            && System.currentTimeMillis() - lastMoveTime >= 500 && lastHalf) {

            robot.leftFront.setTargetPosition(0);
            robot.leftBack.setTargetPosition(0);
            robot.rightFront.setTargetPosition(0);
            robot.rightBack.setTargetPosition(0);

            drivePower = -0.5;

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);
        } */
        
        /*        if (robot.leftFront.getCurrentPosition() < 1250 && robot.leftFront.getCurrentPosition() >= targetPos &&
            robot.rightBack.getCurrentPosition() > 750) {
            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(0.5);
            robot.rightFront.setPower(-0.5);
            robot.rightBack.setPower(-0.5);

            robot.leftFront.setTargetPosition(1250);
            robot.leftBack.setTargetPosition(1250);
            robot.rightFront.setTargetPosition(750);
            robot.rightBack.setTargetPosition(750);
        }
*/
    }
}
