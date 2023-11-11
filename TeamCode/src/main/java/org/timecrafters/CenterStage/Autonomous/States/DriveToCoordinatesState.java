package org.timecrafters.CenterStage.Autonomous.States;

import com.arcrobotics.ftclib.controller.PIDController;

import org.timecrafters.CenterStage.Common.PrototypeRobot;

import dev.cyberarm.engine.V2.CyberarmState;

public class DriveToCoordinatesState extends CyberarmState {

    private final boolean stateDisabled;
    PrototypeRobot robot;
    private PIDController driveXPidController;
    private PIDController driveYPidController;
    private PIDController driveHPidController;
    private double pidX;
    private double pidY;
    private double pidH;

    public double xTarget;
    public double yTarget;
    public double hTarget;

    public double currentVelocityX;
    public double currentVelocityY;
    public double currentVelocityH;

    public double targetVelocityFL;
    public double targetVelocityBL;
    public double targetVelocityFR;
    public double targetVelocityBR;


    public static double Xp = 0, Xi = 0, Xd = 0; // p = 0.02  i = 0.09  d = 0
    public static double Yp = 0, Yi = 0, Yd = 0;// p = 0.03  i = 0.07  d = 0
    public static double Hp = 0, Hi = 0, Hd = 0;// p = 0.03  i = 0.07  d = 0

    public DriveToCoordinatesState(PrototypeRobot robot, String groupName, String actionName) {
        this.robot = robot;
        driveXPidController = new PIDController(Xp, Xi, Xd);
        driveYPidController = new PIDController(-Yp, -Yi, -Yd);
        driveHPidController = new PIDController(-Hp, -Hi, -Hd);
        this.stateDisabled = !robot.configuration.action(groupName, actionName).enabled;
        this.xTarget = robot.configuration.variable(groupName, actionName, "xTarget").value();
        this.yTarget = robot.configuration.variable(groupName, actionName, "yTarget").value();
        this.hTarget = robot.configuration.variable(groupName, actionName, "hTarget").value();
    }

    @Override
    public void start() {
        //add variables that need to be reinitillized
    }

    @Override
    public void exec() {
        // PID functions to come up with the velocity to get to the final point.
        driveXPidController.setPID(Xp, Xi, Xd);
        pidX = driveXPidController.calculate(robot.positionX, xTarget);
        driveYPidController.setPID(Yp, Yi, Yd);
        pidY = (driveYPidController.calculate(robot.positionY, yTarget));
        driveHPidController.setPID(Hp, Hi, Hd);
        pidH = (driveYPidController.calculate(robot.positionH, hTarget));

        // PID robot Velocity Ratios
        currentVelocityX = robot.MaxVelocityForward * pidX;
        currentVelocityY = robot.MaxStrafeVelocity * pidY;
        currentVelocityH = robot.MaxRotationalVelocity * pidH;

        // Kinematic Formulas with plugged in velocities to solve for wheel velocities.
        targetVelocityFL = currentVelocityX - currentVelocityY - ((2 * (robot.L / 2) * robot.MaxRotationalVelocity));
        targetVelocityBL = currentVelocityX + currentVelocityY - ((2 * (robot.L / 2) * robot.MaxRotationalVelocity));
        targetVelocityBR = currentVelocityX - currentVelocityY + ((2 * (robot.L / 2) * robot.MaxRotationalVelocity));
        targetVelocityFR = currentVelocityX + currentVelocityY + ((2 * (robot.L / 2) * robot.MaxRotationalVelocity));

//        robot.frontLeft.setVelocity((targetVelocityFL / robot.R));
//        robot.backLeft.setVelocity((targetVelocityBL / robot.R));
//        robot.backRight.setVelocity((targetVelocityBR / robot.R));
//        robot.frontRight.setVelocity((targetVelocityFR / robot.R));

//        setHasFinished(true)
    }
}
