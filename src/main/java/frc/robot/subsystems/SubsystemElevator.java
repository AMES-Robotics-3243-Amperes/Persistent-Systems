// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.test.SubsystemBaseTestable;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;

import static frc.robot.Constants.Elevator.*;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SubsystemElevator extends SubsystemBaseTestable {

  // H! Uses the parent class for spark maxes and spark flexes, so that they can be easily swapped. REV is the best (:
  private SparkBase motorLeader;
  private SparkBase motorFollower;

  private RelativeEncoder encoderLeader;
  @SuppressWarnings("unused")
  private RelativeEncoder encoderFollower;

  private SparkClosedLoopController controlLoop;
  private double currentReference = 0.0;

  private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

  /** Creates a new SubsystemElevator. */
  public SubsystemElevator() {
    motorLeader = new SparkMax(Motors.leaderCanId, MotorType.kBrushless);
    motorFollower = new SparkMax(Motors.followerCanId, MotorType.kBrushless);

    // Create configuration
    SparkBaseConfig leaderConfig = new SparkFlexConfig();
    SparkBaseConfig followerConfig = new SparkFlexConfig();

    leaderConfig.encoder
      .positionConversionFactor(Motors.positionConversionRatio)
      .velocityConversionFactor(Motors.velocityConversionRatio);
    
    followerConfig.encoder
      .positionConversionFactor(Motors.positionConversionRatio)
      .velocityConversionFactor(Motors.velocityConversionRatio);
    
    leaderConfig.closedLoop
      .pidf(Motors.P, Motors.I, Motors.D, Motors.FF)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    followerConfig.follow(motorLeader, true);

    // Apply configuration
    motorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set encoders
    encoderLeader = motorLeader.getEncoder();
    encoderFollower = motorFollower.getEncoder();

    // Set PID loop
    controlLoop = motorLeader.getClosedLoopController();

    rezero();

    tab.addDouble("Elevator Position", this::getPosition);
    tab.addDouble("Elevator Velocity", this::getVelocity);
    tab.addDouble("Elevator Target", () -> currentReference);
  }

  @Override
  public void doPeriodic() {
    // This method will be called once per scheduler run
  }

  public void nudge(double amount) {
    setPosition(currentReference + amount);
  }

  public void rezero() {
    encoderLeader.setPosition(Positions.starting);
    encoderFollower.setPosition(Positions.starting);
    setPosition(Positions.starting);
  }

  public double getCurrent() {
    return (motorLeader.getOutputCurrent() + motorFollower.getOutputCurrent()) / 2.0;
  }

  // ### Public API used by commands ###

  public void setPosition(double position) {
    position = clamp(Positions.min, Positions.max, position);
    currentReference = position;
    System.out.println("positioning stuff ########");
    System.out.println(position);
    controlLoop.setReference(position / 2.0, ControlType.kPosition); // Cursed offset, do not question the REV gods
  }

  public double getPosition() {
    return encoderLeader.getPosition();
  }

  public double getVelocity() {
    return encoderLeader.getVelocity();
  }

  // TODO H! move to utils class to make nicer
  private static double clamp(double min, double max, double x) {
    if (x < min) { return min; }
    if (x > max) { return max; }
    return x;
  }

  // ### Integration and E2E tests ###

  // Motor Test

  private Future<Boolean> motorTestUserQuestion;
  private boolean motorTestDone = false;

  private void motorTest1() {
    motorLeader.set(0.1);
  }
  private void motorTest2() {
    motorLeader.set(-0.1);
  }
  private void motorTest3() {
    motorLeader.set(0.0);
    motorTestDone = false;
    motorTestUserQuestion = TestUtil.askUserBool("Did both elevators move up and down?");
  }
  private void motorTest4() {
    if (motorTestUserQuestion.isDone()) {
      try {
        if (motorTestUserQuestion.get()) {
          motorTestDone = true;
        } else {
          throw new AssertionError("Motors did not move.");
        }
      } catch (CancellationException | ExecutionException | InterruptedException e) {
        throw new AssertionError(e);
      }
    }
  }

  Test motorTest = new TestUtil.CombinedTest(
    new Test[] {
      new TestUtil.TimedTest(
        new Runnable[] {
          this::motorTest1,
          this::motorTest2,
          this::motorTest3
        }, new double[] {
          1.5,
          1.5,
          0
        }, "Motor Test 1-3"
      ),
      new TestUtil.OnePhaseTest(this::motorTest4, () -> motorTestDone, "Motor Test 4")
    }, 
    "Motor Test"
  );

  // Encoder Test

  double startingPosition = 0.0;

  private void encoderTest1() {
    startingPosition = getPosition();
  }
  private void encoderTest2() {
    motorLeader.set(0.1);
  }
  private void encoderTest3() {
    motorLeader.set(0.0);

    double change = getPosition() - startingPosition;

    if (change < -0.1) {
      throw new AssertionError("Encoder/Motor is backward.");
    }
    if (change < 0.1) {
      throw new AssertionError("Encoder is not detecting more than 0.1 motion.");
    }
  }

  Test encoderTest = new TestUtil.TimedTest(
    new Runnable[] {
      this::encoderTest1,
      this::encoderTest2,
      this::encoderTest3
    },
    new double[] {
      0,
      1.5,
      0
    },
    "Encoder Test",
    new Test[] {
      motorTest
    }
  );

  // Position test

  private void positionTest1() {
    setPosition(Positions.min);
  }
  private void positionTest2() {
    TestUtil.assertEquals(getPosition(), Positions.min, "Did not reach min position.", 0.05);
    setPosition(Positions.max);
  }
  private void positionTest3() {
    TestUtil.assertEquals(getPosition(), Positions.max, "Did not reach max position", 0.05);
    setPosition((Positions.max + Positions.min) / 2.0);
  }
  private void positionTest4() {
    TestUtil.assertEquals(getPosition(), (Positions.max + Positions.min) / 2.0, "Did not reach mid position", 0.05);
  }

  private Test positionTest =
  new TestUtil.TimedTest(
    new Runnable[] {
      this::positionTest1,
      () -> {},
      this::positionTest2,
      () -> {},
      this::positionTest3,
      () -> {},
      this::positionTest4
    },
    new double[] {
      0.0,
      2.0,
      0.0,
      2.0,
      0.0,
      2.0,
      0.0
    },
    "Position Test",
    new Test[] {
      encoderTest,
      motorTest
    }
  );

  // Test collection

  private Test[] tests = new Test[] {
    positionTest
  };

  @Override
  public Test[] getTests() {
    return tests;
  }
}
