package org.timecrafters.CenterStage.Autonomous.States;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoSecDriveState extends CyberarmState {

    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;
    private double drivePower, drivePowerRaw;
    public int readyToTurn, neededTicks, currentTicks, targetTicks = 500;

    public SodiPizzaAutoSecDriveState() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }
    
    private double getDrivePower() {
            if (Math.abs(neededTicks) > 1) {
                drivePower = (drivePowerRaw * neededTicks) / 10;
            }
            return drivePower;
    }

    public void CalculateNeededTicks() {
        if (targetTicks >= 0 && currentTicks >= 0) {
            neededTicks = Math.abs(targetTicks - currentTicks);
        } else if (targetTicks < 0 && currentTicks < 0) {
            neededTicks = Math.abs(targetTicks - currentTicks);
        } else if (targetTicks > 0 && currentTicks < 0) {
            neededTicks = (targetTicks + Math.abs(currentTicks));
        } else if (targetTicks < 0 && currentTicks > 0) {
            neededTicks = (currentTicks + Math.abs(targetTicks));
        }
    }

    @Override
    public void start() {
        
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void exec() {

        readyToTurn = engine.blackboardGet("readyToTurn");

        currentTicks = robot.leftFront.getCurrentPosition();

        CalculateNeededTicks();

        // Move forward from 0 to targetPos
        if (robot.leftFront.getCurrentPosition() <= 10 && readyToTurn == 2) {

            robot.leftFront.setTargetPosition(targetTicks);
            robot.leftBack.setTargetPosition(targetTicks);
            robot.rightFront.setTargetPosition(targetTicks);
            robot.rightBack.setTargetPosition(targetTicks);

            drivePowerRaw = 0.5;
            getDrivePower();

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

        }
        //Stop and finish set after return to 0
        else if (robot.leftFront.getCurrentPosition() >= targetTicks - 10 && robot.leftFront.getCurrentPosition() <= targetTicks + 10) {

            drivePowerRaw = 0;
            getDrivePower();

            robot.leftFront.setPower(drivePower);
            robot.leftBack.setPower(drivePower);
            robot.rightFront.setPower(drivePower);
            robot.rightBack.setPower(drivePower);

            engine.blackboardSet("readyToTurn", 3);

            setHasFinished(true);
        }
    }
}
