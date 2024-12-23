// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.Random;
import java.util.function.Supplier;

import frc.robot.test.TestUtil.InstantTestMethod;

public class TestSubsystem extends SubsystemBaseTestable {

  public double a = 1.3;

  /** Creates a new TestSubsystem. */
  public TestSubsystem() {}

  @Override
  public void doPeriodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public String getName() {
    return "Test Subsystem";
  }

  public class ExampleTest1 implements Test {

    double timer = 0;

    @Override
    public void periodic() {
      timer++;
      a++;
      assert a == timer;
    }

    @Override
    public boolean isDone() {
      return timer == 10;
    }

    @Override
    public void setup() {
      a = 0;
      timer = 0;
    }

    @Override
    public String getName() {
      return "ExampleTest1";
    }
  }

  
  protected void exampleTest2() {
    TestUtil.assertEquals(1+1, 2);
  }

  protected Test exampleSinglePhaseTest = new TestUtil.OnePhaseTest(
    this::exampleSinglePhaseTestMainLoop, 
    this::exampleSinglePhaseTestIsDone, 
    "Example One Phase Test"
  );
  protected void exampleSinglePhaseTestMainLoop() {
    a = new Random().nextInt(1, 7);
  }
  protected boolean exampleSinglePhaseTestIsDone() {
    return a==3;
  }

  @SuppressWarnings("unchecked")
  protected Test multiphaseTest = new TestUtil.MultiphaseTest(
    new Runnable[] {this::phaseOneMain, this::phaseTwoMain}, 
    new Supplier[] {this::phaseOneDone, this::phaseTwoDone}, 
    "Multiphase Test"
  );
  protected void phaseOneMain() {
    a = new Random().nextInt(1, 11);
  }
  protected void phaseTwoMain() {
    a = new Random().nextInt(1, 5);
    throw new AssertionError("Problem!");
  }
  protected boolean phaseOneDone() {
    return a==9;
  }
  protected boolean phaseTwoDone() {
    return a==1;
  }

  protected Test exampleTest2 = new TestUtil.InstantTest(
    this::exampleTest2, 
    "Example Test 2",
    new Test[] {exampleSinglePhaseTest, multiphaseTest}
  );


  @InstantTestMethod(name = "exampleAnnotationTestNamed")
  public void exampleAnnotationTest() {
    TestUtil.assertEquals(1+1+1, 2);
  }

  @InstantTestMethod()
  public void exampleAnnotationTestUnnamed() {
    TestUtil.assertEquals(1+1, 2);
  }

  Test exampleTest1 = new ExampleTest1();

  @Override
  public Test[] getTests() {
    return new Test[] {
      exampleTest1,
      exampleTest2
    };
  }
}
