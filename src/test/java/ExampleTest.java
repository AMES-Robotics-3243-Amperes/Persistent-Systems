import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ExampleTest {
  private long number = 5;

  @BeforeAll
  public void before() {
    assertEquals(5, number);
    number = 10;
  }

  @Test
  public void test1() {
    assertEquals(2, 1 + 1);
    number += 5;
  }

  @Test
  public void test2() {
    assertNotEquals(3, 2 + 2);
    number += 1;
  }

  @AfterAll
  public void after() {
    assertEquals(16, number);
  }
}
