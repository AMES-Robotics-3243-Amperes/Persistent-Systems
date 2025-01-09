import org.junit.jupiter.api.AssertionFailureBuilder;

import edu.wpi.first.math.geometry.Translation2d;

public class AssertHelpers {
  private static <T> void failNotEqual(T expected, T actual, String message) {
    AssertionFailureBuilder.assertionFailure()
        .message(message)
        .expected(expected)
        .actual(actual)
        .buildAndThrow();
  }

  public static <T> void assertEqualsOr(T expectedOne, T expectedTwo, T actual) {
    if (expectedOne != actual && expectedTwo != actual) {
      AssertionFailureBuilder.assertionFailure().buildAndThrow();
    }
  }

  public static void assertEqualsOr(double expectedOne, double expectedTwo, double actual, double delta) {
    if (Math.abs(expectedOne - actual) > delta && Math.abs(expectedTwo - actual) > delta) {
      AssertionFailureBuilder.assertionFailure().buildAndThrow();
    }
  }

  public static void assertEqualsOr(Translation2d expectedOne, Translation2d expectedTwo, Translation2d actual,
      double delta) {
    if (expectedOne.getDistance(actual) > delta && expectedTwo.getDistance(actual) > delta) {
      AssertionFailureBuilder.assertionFailure().buildAndThrow();
    }
  }

  public static void assertEquals(Translation2d expected, Translation2d actual, double delta) {
    if (expected.getDistance(actual) > delta) {
      failNotEqual(expected, actual, null);
    }
  }

  public static void assertEquals(Translation2d expected, Translation2d actual, String message) {
    if (expected != actual) {
      failNotEqual(expected, actual, message);
    }
  }

  public static void assertEquals(Translation2d expected, Translation2d actual, double delta, String message) {
    if (expected.getDistance(actual) > delta) {
      failNotEqual(expected, actual, message);
    }
  }
}
