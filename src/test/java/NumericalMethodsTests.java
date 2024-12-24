import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.splines.NumericalMethods;
import frc.robot.splines.NumericalMethods.DifferentiableFunction;

public class NumericalMethodsTests {
  @Test
  public void newtonRaphson() {
    DifferentiableFunction f = new DifferentiableFunction() {

      @Override
      public double sample(double x) {
        return x * x * x - (x - 1) * Math.sin(x);
      }

      @Override
      public double firstDerivative(double x) {
        return 3 * x * x - (x - 1) * Math.cos(x) - Math.sin(x);
      }

    };

    double approximateSolutionOne = NumericalMethods.newtonRaphson(f, 1, 6);
    assertEquals(0, f.sample(approximateSolutionOne), 1e-9);

    double approximateSolutionTwo = NumericalMethods.newtonRaphson(f, -1, 6);
    assertEquals(0, f.sample(approximateSolutionTwo), 1e-9);
  }

  @Test
  public void gaussianQuadrature() {
    assertEquals(2, NumericalMethods.gaussianQuadrature(Math::sin, 0, Math.PI), 1e-3);
    assertEquals(Math.exp(5) - 1, NumericalMethods.gaussianQuadrature(Math::exp, 0, 5), 1e-3);
  }

  @Test
  public void compositeGaussianQuadrature() {
    assertEquals(2, NumericalMethods.compositeGaussianQuadrature(Math::sin, 0, Math.PI, 5), 1e-4);
    assertEquals(Math.exp(5) - 1, NumericalMethods.compositeGaussianQuadrature(Math::exp, 0, 5, 5), 1e-4);
  }
}
