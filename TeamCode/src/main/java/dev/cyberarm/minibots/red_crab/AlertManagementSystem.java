package dev.cyberarm.minibots.red_crab;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.Comparator;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

public class AlertManagementSystem {
    private CopyOnWriteArrayList<Alert> alerts = new CopyOnWriteArrayList<>();

    public enum Priority {
        CRITICAL, // Something is on "fire"
        WARNING, // Something is soon to be on "fire"
        ALERT, // Something isn't quite right
        NOTICE // Informational message
    }

    public  AlertManagementSystem() {
    }

    public void report(Telemetry telemetry) {
        if (alerts.size() == 0) {
            // Pad a few lines so telemetry doesn't jump about (as much)
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine();
            return;
        }

        telemetry.addLine("------ ALERT MANAGEMENT SYSTEM ------");
        alerts.stream().sorted(Comparator.comparing(Alert::getPriority)).filter(Alert::valid).forEach(
                alert -> telemetry.addData("", "    %s: %s", alert.priority.name(), alert.message));
        telemetry.addLine("-------------------------------------------------------------"); // Gotta love non-monospace fonts, eh?

        alerts.stream().filter(Alert::expired).forEach(alert -> alerts.remove(alert));
    }

    public void addCritical(String category, String message) {
        alerts.add(
                new Alert(Priority.CRITICAL, category, message)
        );
    }

    public void addWarning(String category, String message) {
        alerts.add(
                new Alert(Priority.WARNING, category, message)
        );
    }

    public void addAlert(String category, String message) {
        alerts.add(
                new Alert(Priority.ALERT, category, message)
        );
    }

    public void addNotice(String category, String message) {
        alerts.add(
                new Alert(Priority.NOTICE, category, message)
        );
    }

    public class Alert {
        private final Priority priority;
        private final String category;
        private final String message;
        private final long timeToLive;
        private final long bornAt;
        private Alert(Priority priority, String category, String message) {
            this.priority = priority;
            this.category = category;
            this.message = message;

            this.bornAt = System.currentTimeMillis();
            this.timeToLive = 5_000;
        }

        private int getPriority() {
            return priority.ordinal();
        }

        private boolean expired() {
            return System.currentTimeMillis() - bornAt >= timeToLive;
        }

        private boolean valid() {
            return !expired();
        }
    }
}
