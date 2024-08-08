package frc.robot.splines;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CurveConstants;

/**
 * Used to represent information along a spline, including robot
 * rotation and velocity.
 * 
 * TODO: extend this with an arclength defined version
 * 
 * @author :3
 */
public class SplineMetadata {
  /**
   * Constructs a new SplineMetadata with sensible defaults.
   * 
   * @author :3
   */
  public SplineMetadata() {}

  /**
   * Serves as a single pice of parameterization-dependent metadata.
   */
  public class Metadata<T> {
    private Function<Double, T> parameterization;
    private boolean active;

    /**
     * Takes in a constant value and a boolean for being enabled.
     * 
     * @author :3
     */
    public Metadata(T value, boolean active) {
      this.parameterization = x -> value;
      this.active = active;
    }

    /**
     * Sets metadata to a constant
     * 
     * @param parameterization the value for the metadata to take
     * 
     * @author :3
     */
    public void set(T parameterization) {
      this.parameterization = x -> parameterization;
      active = true;
    }

    /**
     * Sets metadata to a function
     * 
     * @param parameterization the value for the metadata to take
     * 
     * @author :3
     */
    public void set(Function<Double, T> parameterization) {
      this.parameterization = parameterization;
      active = true;
    }

    /**
     * Gets the metadata at a parameterization
     * 
     * @param t the parameterization to get the metadata at
     * 
     * @author :3
     */
    public T get(double t) {
      return parameterization.apply(t);
    }

    /**
     * Gets if the metadata is active
     * 
     * @author :3
     */
    public boolean active() {
      return active;
    }
  }

  public Metadata<Double> velocity = new Metadata<Double>(CurveConstants.baseVelocity, true);
  public Metadata<Rotation2d> rotation = new Metadata<Rotation2d>(new Rotation2d(), false);
}
