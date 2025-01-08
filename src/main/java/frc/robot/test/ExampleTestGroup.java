package frc.robot.test;

public class ExampleTestGroup implements TestGroup {

  @Override
  public String getName() {
    return "Example Test Group";
  }

  @Override
  public Test[] getTests() {
    return new Test[]{instanceExampleFailingTest, new ExampleSuperDependentTest()};
  }

  protected class ExampleTest implements Test {
  @Override public void periodic() {/*TestUtil.assertEquals(2,3);*/}
    @Override public boolean isDone() {return true;}

    @Override public String getName() {return "Example Test";}

    @Override public Test[] getDependencies() {return new Test[0];}
  }

  protected class ExampleFailingTest implements Test {

    @Override public void periodic() {TestUtil.assertEquals(2+2,7);}
    @Override public boolean isDone() {return true;}

    @Override public String getName() {return "Example Failing Test";}

    @Override public Test[] getDependencies() {return new Test[0];}
  }
  public ExampleFailingTest instanceExampleFailingTest = new ExampleFailingTest();

  protected class ExampleDependentTest implements Test {
    @Override public void periodic() {}
    @Override public boolean isDone() {return true;}

    @Override public String getName() {return "Example Dependent Test";}

    protected Test[] dependencies = new Test[]{new ExampleTest(), instanceExampleFailingTest};

    @Override public Test[] getDependencies() {return dependencies;}
    /*@Override public boolean[] getDependencySuccessRequirements() {
      return new boolean[]{true, false};
    }*/
  }

  protected class ExampleSuperDependentTest implements Test {
    @Override public void periodic() {}
    @Override public boolean isDone() {return true;}

    @Override public String getName() {return "Example Super Dependent Test";}

    protected Test[] dependencies = new Test[]{new ExampleDependentTest()};

    @Override public Test[] getDependencies() {return dependencies;}
  }
    
}
