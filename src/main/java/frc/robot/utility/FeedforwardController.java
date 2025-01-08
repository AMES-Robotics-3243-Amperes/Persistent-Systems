package frc.robot.utility;

public interface FeedforwardController {
    public double calculateVoltage(double velocity, double acceleration);
}