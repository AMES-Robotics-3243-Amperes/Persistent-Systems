// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubsystemClaw extends SubsystemBase {

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
      
      double maxOuput = Math.max(mechanismInputs.get(0), mechanismInputs.get(1));
      if (maxOuput > 1.0) {
        mechanismInputs.div(maxOuput);
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

  public boolean isAtTarget() {
    return targetPivotPosition == pivotEncoder.getDistance();
  }

  /** Creates a new SubsystemEndAffectorDifferential. */
  public SubsystemClaw() {
    motorGroup = new DifferentialMotorGroup(motor1, motor2);
    pivotController = new PIDController(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorGroup.setOutsideOutput(pivotController.calculate(pivotEncoder.getDistance(), targetPivotPosition));
    motorGroup.setInsideOutput(intakePower);
    motorGroup.update();
  }

  private static double clamp(double min, double max, double x) {
    return Math.max(min, Math.min(max, x));
  }

  public enum SetpointDiffArm {
    Starting(Constants.DifferentialArmConstants.Setpoints.startingPosition, Constants.DifferentialArmConstants.Setpoints.startingPower),
    Intake(Constants.DifferentialArmConstants.Setpoints.intakePosition, Constants.DifferentialArmConstants.Setpoints.intakePower),
    Place(Constants.DifferentialArmConstants.Setpoints.depositPosition, Constants.DifferentialArmConstants.Setpoints.depositPower);

    public final double position;
    public final double power;

    private SetpointDiffArm(double position, double power) {
      this.position = position;
      this.power = power;
    }
  }
}
