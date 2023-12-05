package org.timecrafters.CenterStage.Autonomous.SodiStates;

import android.annotation.SuppressLint;

import org.timecrafters.CenterStage.Common.SodiPizzaMinibotObject;

import dev.cyberarm.engine.V2.CyberarmState;

public class SodiPizzaWheelTest extends CyberarmState {
    private SodiPizzaMinibotObject robot;
    private String groupName, actionName;
    private long lastMoveTime;
    private double lDrivePower, rDrivePower;

    public SodiPizzaWheelTest() {
        groupName = " ";
        actionName = " ";
        robot = new SodiPizzaMinibotObject();
        robot.setup();
    }
    
    private double getrDrivePower() {
        rDrivePower = lDrivePower * 0.75;
        return rDrivePower;
    }

    @Override
    public void telemetry() {

        engine.telemetry.addData("Current Pos? ", robot.leftFront.getCurrentPosition());
        engine.telemetry.addData("Last Moved Time? ", lastMoveTime);
        engine.telemetry.addData("System Current Time? ", System.currentTimeMillis());

        engine.telemetry.addData("Diff in Right/Left Front Wheel Power? ", robot.rightFront.getPower() - robot.leftFront.getPower());
    }

    @Override
    public void start() {

        lastMoveTime = System.currentTimeMillis();
        lDrivePower = 0;
        rDrivePower = 0;
        robot.leftFront.setPower(lDrivePower);
        robot.leftBack.setPower(lDrivePower);
        robot.rightFront.setPower(rDrivePower);
        robot.rightBack.setPower(rDrivePower);
        
    }

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void exec() {

/*        lDrivePower = 0.3;
        robot.leftFront.setPower(lDrivePower);
        robot.leftBack.setPower(lDrivePower);
        robot.rightFront.setPower(lDrivePower);
        robot.rightBack.setPower(lDrivePower);
*/
        if (robot.leftFront.getCurrentPosition() <= 10 && robot.leftFront.getCurrentPosition() >= -10 && System.currentTimeMillis() - lastMoveTime >= 3000) {
                robot.leftFront.setTargetPosition(500);
                robot.leftBack.setTargetPosition(500);
                robot.rightFront.setTargetPosition(500);
                robot.rightBack.setTargetPosition(500);

                lDrivePower = 0.3;
                rDrivePower = lDrivePower * 0.75;

                robot.leftFront.setPower(lDrivePower);
                robot.leftBack.setPower(lDrivePower);
                robot.rightFront.setPower(rDrivePower);
                robot.rightBack.setPower(rDrivePower);

                lastMoveTime = System.currentTimeMillis();

        } else if (robot.leftFront.getCurrentPosition() >= 500 && System.currentTimeMillis() - lastMoveTime >= 3000) {
                    robot.leftFront.setTargetPosition(0);
                    robot.leftBack.setTargetPosition(0);
                    robot.rightFront.setTargetPosition(0);
                    robot.rightBack.setTargetPosition(0);

                    lDrivePower = -0.3;
                    rDrivePower = lDrivePower * 0.75;

                    robot.leftFront.setPower(lDrivePower);
                    robot.leftBack.setPower(lDrivePower);
                    robot.rightFront.setPower(rDrivePower);
                    robot.rightBack.setPower(rDrivePower);

                    lastMoveTime = System.currentTimeMillis();
        }
    }
}
