// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemEndAffectorDifferential extends SubsystemBase {

  private SparkBase motor1;
  private SparkBase motor2;
  private DifferentialMotorGroup motorGroup;
  private PIDController pivotController;

  private static double startingPivotPosition = 0.0;
  private double targetPivotPosition = startingPivotPosition;
  private double intakePower = 0.0;

  private Encoder pivotEncoder;

  private static class DifferentialMotorGroup {
    private MotorController motorForward;
    private MotorController motorReverse;

    private double outsideOutput;
    private double insideOutput;

    private static Matrix<N2, N2> inverseDifferentialMatrix = new Matrix<N2, N2>(N2.instance, N2.instance, new double[] {
       1.0, 1.0,
      -1.0, 1.0
    });

    public DifferentialMotorGroup(MotorController motorForward, MotorController motorReverse) {
      this.motorForward = motorForward;
      this.motorReverse = motorReverse;
    }

    private void update() {
      Matrix<N2, N1> mechanismOutputs = new Matrix<N2, N1>(N2.instance, N1.instance, new double[] {outsideOutput, insideOutput});
      Vector<N2> mechanismInputs = new Vector<N2>(inverseDifferentialMatrix.times(mechanismOutputs));
      
      // Normalize output if not achievable
      if (mechanismInputs.get(0) > 1.0) {
        mechanismInputs = mechanismInputs.div(mechanismInputs.get(0));
      }

      if (mechanismInputs.get(1) > 1.0) {
        mechanismInputs = mechanismInputs.div(mechanismInputs.get(1));
      }

      motorForward.set(mechanismInputs.get(0));
      motorReverse.set(mechanismInputs.get(1));
    }

    public void setOutsideOutput(double speed) {
      outsideOutput = clamp(-1.0, 1.0, speed);
      update();
    }

    public void setInsideOutput(double speed) {
      insideOutput = clamp(-1.0, 1.0, speed);
      update();
    }
  }

  public void setOutsidePosition(double position) {
    targetPivotPosition = position;
  }

  public void setIntakePower(double power) {
    intakePower = power;
  }

  /** Creates a new SubsystemEndAffectorDifferential. */
  public SubsystemEndAffectorDifferential() {
    motorGroup = new DifferentialMotorGroup(motor1, motor2);
    pivotController = new PIDController(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorGroup.setOutsideOutput(pivotController.calculate(pivotEncoder.getDistance(), targetPivotPosition));
    motorGroup.setInsideOutput(intakePower);
  }

  private static double clamp(double min, double max, double x) {
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
  }
}
