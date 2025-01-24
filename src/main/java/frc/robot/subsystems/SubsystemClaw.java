// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Decide if we want to use switch, or ultrasonic sensor, etc.
import edu.wpi.first.wpilibj.Ultrasonic;


import frc.robot.Constants.DifferentialArm.*;

public class SubsystemClaw extends SubsystemBase {
  public Ultrasonic rangeFinder;

  private SparkMax forwardMotor = new SparkMax(MotorIDs.forwardId, MotorType.kBrushless);
  private SparkMax reverseMotor = new SparkMax(MotorIDs.reverseId, MotorType.kBrushless);
  
  private SparkMaxConfig forwardConfig = new SparkMaxConfig();
  private SparkMaxConfig reverseConfig = new SparkMaxConfig();

  private DifferentialMotorGroup motorGroup;
  private PIDController pivotController;

  private static double startingPivotPosition = 0.0;
  private double targetPivotPosition = startingPivotPosition;
  private double intakePower = 0.0;

  private AbsoluteEncoder pivotEncoder;

  private static class DifferentialMotorGroup {
    private MotorController motorForward;
    private MotorController motorReverse;

    private double outsideOutput;
    private double insideOutput;

    private static Matrix<N2, N2> inverseDifferentialMatrix = new Matrix<N2, N2>(N2.instance, N2.instance, new double[] {
       1.0, 1.0,
       1.0, -1.0
    });

    public DifferentialMotorGroup(MotorController motorForward, MotorController motorReverse) {
      this.motorForward = motorForward;
      this.motorReverse = motorReverse;
    }

    private void update() {
      Matrix<N2, N1> mechanismOutputs = new Matrix<N2, N1>(N2.instance, N1.instance, new double[] {outsideOutput, insideOutput});
      Vector<N2> mechanismInputs = new Vector<N2>(inverseDifferentialMatrix.times(mechanismOutputs));
      
      double maxMotorOutput = Math.max(mechanismInputs.get(0), mechanismInputs.get(1));
      if (maxMotorOutput > 1.0) {
        mechanismInputs.div(maxMotorOutput);
      }

      motorForward.set(mechanismInputs.get(0));
      motorReverse.set(mechanismInputs.get(1));
    }

    public void setOutsideOutput(double speed) {
      outsideOutput = clamp(-1.0, 1.0, speed);
    }

    public void setInsideOutput(double speed) {
      insideOutput = clamp(-1.0, 1.0, speed);
    }
  }

  public void setOutsidePosition(double position) {
    targetPivotPosition = position;
  }

  public void setIntakePower(double power) {
    intakePower = power;
  }

  /** Creates a new SubsystemEndAffectorDifferential. */
  public SubsystemClaw(/* Ultrasonic rangeFinder */) {
    // Might need to invert this motor
    // reverseConfig.inverted(true);
    forwardMotor.configure(forwardConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    reverseMotor.configure(reverseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = forwardMotor.getAbsoluteEncoder();

    motorGroup = new DifferentialMotorGroup(forwardMotor, reverseMotor);
    pivotController = new PIDController(0.1, 0, 0);

    // Decide if we want to use switch, or ultrasonic sensor, etc.
    // this.rangeFinder = rangeFinder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorGroup.setOutsideOutput(pivotController.calculate(pivotEncoder.getPosition(), targetPivotPosition));
    motorGroup.setInsideOutput(intakePower);
    motorGroup.update();
    System.out.println("Intake power: " + intakePower);
    System.out.println("Motor position: " + targetPivotPosition);
    System.out.println("Absolute encoder value: " + pivotEncoder.getPosition());
  }

  private static double clamp(double min, double max, double x) {
    return Math.max(min, Math.min(max, x));
  }

  public enum SetpointDiffArm {
    Starting(Setpoints.startingPosition, Setpoints.startingPower),
    Intake(Setpoints.intakePosition, Setpoints.intakePower),
    Place(Setpoints.depositPosition, Setpoints.depositPower);

    public final double position;
    public final double power;

    private SetpointDiffArm(double position, double power) {
      this.position = position;
      this.power = power;
    }
  }
}
