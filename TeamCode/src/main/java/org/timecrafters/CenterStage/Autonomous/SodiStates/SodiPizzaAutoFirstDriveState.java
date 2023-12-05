package org.timecrafters.CenterStage.Autonomous.SodiStates;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaAutoFirstDriveState extends CyberarmState{
    final private SodiPizzaMinibotObject robot;
    final private String groupName, actionName;
    private long lastMoveTime;
    private int targetPos = 2500;
    private double drivePower;
    public int readyToTurn;

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
        readyToTurn = engine.blackboardGetInt("readyToTurn");

    }

    @Override
    public void telemetry() {
        engine.telemetry.addData("Target Ticks?", robot.leftFront.getTargetPosition());
        engine.telemetry.addData("Current Ticks?", robot.leftFront.getCurrentPosition());
        engine.telemetry.addData("Ticks Needed?", robot.leftFront.getTargetPosition() -
        robot.leftFront.getCurrentPosition());
        engine.telemetry.addLine();
        engine.telemetry.addData("Internal Ready To Turn Value", readyToTurn);
        engine.telemetry.addData("Distance Sensor Reading", robot.distSensor.getDistance(DistanceUnit.MM));

    }

    @Override
    public void exec() {

        readyToTurn = engine.blackboardGet("readyToTurn");

        // Move forward from 0 to targetPos
        if (robot.leftFront.getCurrentPosition() <= 10 && robot.leftFront.getCurrentPosition() >= -10 && readyToTurn == 0) {

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

            engine.blackboardSet("readyToTurn", 1);

            setHasFinished(true);
        }
    }
}
