// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** A class to handle control of a motor using a PID controller running through a feedforward controller.
 * This is often useful, as the feedforward can essentially linearize the system, and PID loops are highly
 * effective at controlling linear systems.
 */
public class PIDFMotorController {

    protected MotorController motor;
    protected FeedforwardController feedforward;
    protected PIDController pidController;
    protected Optional<ArbitraryEncoder> attachedEncoder = Optional.empty();

    /**
     * Constructs a controller.
     * @param motor The motor to control
     * @param pidController The PID controller to use
     * @param feedforward The feedforward controller to use. It is simple to adapt an existing
     * controller type by wrapping it in a class which implements {@link FeedforwardController}
     */
    public PIDFMotorController(MotorController motor, PIDController pidController, FeedforwardController feedforward) {
        this.motor = motor;
        this.pidController = pidController;
        this.feedforward = feedforward;
    }

    public PIDFMotorController(MotorController motor, PIDController pidController, FeedforwardController feedforward, ArbitraryEncoder attachedEncoder) {
        this.motor = motor;
        this.pidController = pidController;
        this.feedforward = feedforward;
        this.attachedEncoder = Optional.of(attachedEncoder);
    }

    public PIDFMotorController(MotorController motor, PIDController pidController, FeedforwardController feedforward, Encoder attachedEncoder) {
        this(motor, pidController, feedforward, new ArbitraryEncoder(attachedEncoder));
    }

    public PIDFMotorController(MotorController motor, PIDController pidController, FeedforwardController feedforward, AbsoluteEncoder attachedEncoder) {
        this(motor, pidController, feedforward, new ArbitraryEncoder(attachedEncoder));
    }

    public PIDFMotorController(MotorController motor, PIDController pidController, FeedforwardController feedforward, RelativeEncoder attachedEncoder) {
        this(motor, pidController, feedforward, new ArbitraryEncoder(attachedEncoder));
    }






    /**
     * Runs the motor according to the provided measurements
     * @param position The current position of the mechanism
     * @param velocity The current velocity of the mechanism
     */
    public void update(double position, double velocity) {
        motor.setVoltage(feedforward.calculateVoltage(velocity, pidController.calculate(position)));
    }
    /**
     * Runs the motor according to the provided measurements
     * @param encoder An encoder which can provide data about the mechanism
     */
    public void update(Encoder encoder) {
        update(encoder.getDistance(), encoder.getRate());
    }
    /**
     * Runs the motor according to the provided measurements
     * @param encoder An encoder which can provide data about the mechanism
     */
    public void update(AbsoluteEncoder encoder) {
        update(encoder.getPosition(), encoder.getVelocity());
    }
    /**
     * Runs the motor according to the provided measurements
     * @param encoder An encoder which can provide data about the mechanism
     */
    public void update(RelativeEncoder encoder) {
        update(encoder.getPosition(), encoder.getVelocity());
    }

    /**
     * Runs the motor according to the measurements of the attached encoder.
     * @throws IllegalArgumentException If the method is called before the attached encoder is set.
     */
    public void update() {
        if (attachedEncoder.isEmpty()) {
            throw new IllegalArgumentException("Attached encoder was used before it was set.");
        }
        update(attachedEncoder.get().getPosition(), attachedEncoder.get().getVelocity());
    }



    /**
     * Sets the attached encoder for the controller.
     * @param encoder
     */
    public void setEncoder(ArbitraryEncoder encoder) {
        attachedEncoder = Optional.of(encoder);
    }
    /**
     * Sets the attached encoder for the controller.
     * @param encoder
     */
    public void setEncoder(Encoder encoder) {
        setEncoder(new ArbitraryEncoder(encoder));
    }
    /**
     * Sets the attached encoder for the controller.
     * @param encoder
     */
    public void setEncoder(AbsoluteEncoder encoder) {
        setEncoder(new ArbitraryEncoder(encoder));
    }
    /**
     * Sets the attached encoder for the controller.
     * @param encoder
     */
    public void setEncoder(RelativeEncoder encoder) {
        setEncoder(new ArbitraryEncoder(encoder));
    }







    /**
     * Sets the target setpoint for which the controller will calculate its movements
     */
    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

}
