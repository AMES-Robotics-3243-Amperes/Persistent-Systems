import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.DataManager;
import frc.robot.RobotContainer;
import frc.robot.DataManager.Entry;

public class DataManagerTests {
  RobotContainer robotContainer = new RobotContainer();

  public class MockEntry extends Entry<Integer> {
    int update_calls = 0;

    public MockEntry() {
      DataManager.instance().super();
    }

    @Override
    public Integer get() {
      return update_calls;
    }

    @Override
    public void update() {
      update_calls++;
    }
  }

  @BeforeAll
  public void resetDataManager() {
    new DataManager(robotContainer);
  }

  @Test
  public void dataMangerInstanceExists() {
    // if this fails, something has gone wrong in the
    // creation of our DataManager instance
    DataManager.instance().update();
  }

  @Test
  public void entriesAutoConnectToDataManagerUpdate() {
    MockEntry entry = new MockEntry();

    assertEquals(Integer.valueOf(0), entry.get());
    DataManager.instance().update();
    assertEquals(Integer.valueOf(1), entry.get());
    
    for (int _i = 0; _i < 10; _i++) DataManager.instance().update();
    assertEquals(Integer.valueOf(11), entry.get());
  }
}
