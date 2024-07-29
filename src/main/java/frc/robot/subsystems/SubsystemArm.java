// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ArbitraryEncoder;
import frc.robot.utility.FeedforwardControllerArm;
import frc.robot.utility.PIDFMotorController;

public class SubsystemArm extends SubsystemBase {

  protected CANSparkMax motor;
  protected ArbitraryEncoder encoder;
  protected PIDFMotorController pidf;

  /** Creates a new SubsystemArm. */
  public SubsystemArm() {
    motor = new CANSparkMax(11, MotorType.kBrushless);
    encoder = new ArbitraryEncoder(motor.getAbsoluteEncoder());
    pidf = new PIDFMotorController(
      motor, 
      new PIDController(0.05, 0, 0), // TODO Tune these
      new FeedforwardControllerArm(0, 0, 0, 0), // TODO Calculate these
      encoder
    );
  }

  public void setPosition(double position) {
    pidf.setSetpoint(position);
  }

  public void setPosition(Setpoint position) {
    setPosition(position.rotations);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public enum Setpoint {
    GROUND(0.065),
    MIDDLE(0.127),
    TOP(0.284);
    public final double rotations;
    private Setpoint(double rotations) {
      this.rotations = rotations;
    }
  }
}
