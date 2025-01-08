package frc.robot.utility;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedforwardControllerArm extends ArmFeedforward implements FeedforwardController {

    /** @see SimpleMotorFeedforward#SimpleMotorFeedforward */
    public FeedforwardControllerArm(double ks, double kg, double kv) {
        super(ks, kg, kv);
    }

    /** @see SimpleMotorFeedforward#SimpleMotorFeedforward */
    public FeedforwardControllerArm(double ks, double kg, double kv, double ka) {
        super(ks, kg, kv, ka);
    }

    @Override
    public double calculateVoltage(double velocity, double acceleration) {
        return calculate(velocity, acceleration);
    }

}
