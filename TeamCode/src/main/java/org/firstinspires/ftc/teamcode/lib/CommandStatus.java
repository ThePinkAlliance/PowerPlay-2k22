package org.firstinspires.ftc.teamcode.lib;

public enum CommandStatus {
    ACCEPTED("ACCEPTED"),
    REJECTED("REJECTED");

    String text;

    CommandStatus(String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return this.text;
    }
}
