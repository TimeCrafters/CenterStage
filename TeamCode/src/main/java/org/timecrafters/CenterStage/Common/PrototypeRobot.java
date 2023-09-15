package org.timecrafters.CenterStage.Common;

import org.timecrafters.Library.Robot;

public class PrototypeRobot extends Robot {
    private String string;
    public PrototypeRobot(String string) {
        this.string = string;
    }
    @Override
    public void setup() {
        System.out.println("Bacon: " + this.string);
    }
}
