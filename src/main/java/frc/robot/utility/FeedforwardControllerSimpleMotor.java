package frc.robot.utility;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedforwardControllerSimpleMotor extends SimpleMotorFeedforward implements FeedforwardController {

    /** @see SimpleMotorFeedforward#SimpleMotorFeedforward */
    public FeedforwardControllerSimpleMotor(double ks, double kv) {
        super(ks, kv);
    }

    /** @see SimpleMotorFeedforward#SimpleMotorFeedforward */
    public FeedforwardControllerSimpleMotor(double ks, double kv, double ka) {
        super(ks, kv, ka);
    }

    @Override
    public double calculateVoltage(double velocity, double acceleration) {
        return calculate(velocity, acceleration);
    }

}
