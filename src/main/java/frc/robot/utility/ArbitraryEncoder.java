package frc.robot.utility;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;

public class ArbitraryEncoder {

    Supplier<Double> positionGetter;
    Supplier<Double> velocityGetter;

    public ArbitraryEncoder(Encoder encoder) {
        positionGetter = encoder::getDistance;
        velocityGetter = encoder::getRate;
    }

    public ArbitraryEncoder(AbsoluteEncoder encoder) {
        positionGetter = encoder::getPosition;
        velocityGetter = encoder::getVelocity;
    }

    public ArbitraryEncoder(RelativeEncoder encoder) {
        positionGetter = encoder::getPosition;
        velocityGetter = encoder::getVelocity;
    }






    public double getPosition() {
        return positionGetter.get();
    }

    public double getVelocity() {
        return velocityGetter.get();
    }
}
