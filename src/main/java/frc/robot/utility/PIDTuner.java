package frc.robot.utility;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PIDTuningConstants;

/**
 * Automatically tunes PIDs, and prints the found "optimal" values to the
 * console. Use in development, and store the found values somewhere external
 * to this class (such as {@link Constants}).
 * 
 * @author :3
 */
public class PIDTuner extends Command {
  // methods and values relating to the PID and system
  private final Consumer<Double> setPIDTarget;
  private final Supplier<Double> getSystemOutput;

  private final Consumer<Double> setP;
  private final Consumer<Double> setI;
  private final Consumer<Double> setD;

  private final SystemInformation systemInformation;

  // members used throughout tuning
  private Timer timer;
  private Step step;
  private double Kp;

  // step-specific tuning members
  private double startMeasurement;
  private double previousMeasurement;
  private double currentTarget;

  private boolean positiveChange;

  /**
   * Represents the different steps of finding Ku
   */
  private enum Step {
    Setup,
    Running,
    Oscillating,
    Measuring,
    Resetting
  }

  /**
   * Information about potential system outputs. See constructor for
   * more specific information about what to provide.
   * @author :3
   */
  public class SystemInformation {
    private double baseline;
    private double reasonableTarget;

    /**
     * Creates a new SpeedValues
     * 
     * @param baseline the "halfway" point of the system: velocity zero, position halfway between max and min, etc
     * @param reasonableTarget a reasonable initial target to test the system (around halfway between baseline and reaonable maximum output)
     * 
     * @author :3
     */
    public SystemInformation(double baseline, double reasonableTarget) {
      this.baseline = baseline;
      this.reasonableTarget = reasonableTarget;
    }
  }

  public enum TuningStrategies {
    CohenCoon
  }

  /**
   * Creates a new PID tuner
   * 
   * @param setPIDTarget a runnable that sets the target of the PID. the tuner will not run the PID, that needs to be done
   * @param bail a runnable that returns the system to the default state if possible.
   * @param getSystemOutput returns the current system output
   * @param setP sets the P coefficient of the PID controller
   * @param setI sets the I coefficient of the PID controller
   * @param setD sets the D coefficient of the PID controller
   * @param systemInformation a {@link SystemInformation} for the controlled system
   * 
   * @author :3
   */
  public PIDTuner(Consumer<Double> setPIDTarget,
      Supplier<Double> getSystemOutput,
      Consumer<Double> setP,
      Consumer<Double> setI,
      Consumer<Double> setD,
      SystemInformation systemInformation) {
    this.setPIDTarget = setPIDTarget;
    this.getSystemOutput = getSystemOutput;

    this.setP = setP;
    this.setI = setI;
    this.setD = setD;

    this.systemInformation = systemInformation;

    this.timer = new Timer();
  }

  /**
   * Creates a new PID tuner for a velocity PID run by a {@link CANSparkMax}
   * 
   * @param PIDController a PID controller for the controlled {@link CANSparkMax}
   * @param encoder the relative encoder measuring velocity output (usually same motor as PIDController)
   * @param systemInformation a {@link SystemInformation} for the controlled system
   * 
   * @author :3
   */
  public PIDTuner(SparkPIDController PIDController, RelativeEncoder encoder, SystemInformation systemInformation) {
    this((Double PIDTarget) -> PIDController.setReference(PIDTarget, ControlType.kVelocity),
      encoder::getVelocity,
      PIDController::setP,
      PIDController::setI,
      PIDController::setD,
      systemInformation);
  }

  /**
   * Creates a new PID tuner for a position/velocity PID run by a {@link CANSparkMax}
   * 
   * @param PIDController a PID controller for the controlled {@link CANSparkMax}
   * @param encoder the absolute encoder measuring system output (usually same motor as PIDController)
   * @param controlType {@link ControlType} of the PID controller: position or velocity (other values will give strange results)
   * @param systemInformation a {@link SystemInformation} for the controlled system
   * 
   * @author :3
   */
  public PIDTuner(SparkPIDController PIDController, AbsoluteEncoder encoder, ControlType controlType, SystemInformation systemInformation) {
    this((Double PIDTarget) -> PIDController.setReference(PIDTarget, controlType),
      (controlType == ControlType.kPosition) ? encoder::getPosition : encoder::getVelocity,
      PIDController::setP,
      PIDController::setI,
      PIDController::setD,
      systemInformation);
  }

  @Override
  public void initialize() {
    setP.accept(0.0);
    setI.accept(0.0);
    setD.accept(0.0);

    Kp = 10e-5;

    step = Step.Setup;
    timer.stop();
    timer.reset();
  }

  @Override
  public void execute() {
    switch (step) {
      case Setup:
        setup();
        break;
      
      case Running:
        running();
        break;
      
      case Oscillating:
        oscillating();
        break;

      case Measuring:
        measuring();
        break;

      case Resetting:
        resetting();
        break;
    
      default:
        break;
    }
  }

  public void setup() {
    startMeasurement = getSystemOutput.get();
    previousMeasurement = getSystemOutput.get();
    currentTarget = getLoopTarget();

    setPIDTarget.accept(currentTarget);

    timer.restart();
    step = Step.Running;
  }

  public void running() {
    double progress = Math.abs((startMeasurement - getSystemOutput.get()) / (currentTarget - startMeasurement));
    boolean sufficientProgress = progress > timer.get() * PIDTuningConstants.progressCheckProportionConstant;
    boolean progressCheck = timer.get() > PIDTuningConstants.progressCheckDelay;

    if (progress > 1) {
      positiveChange = currentTarget - startMeasurement > 0;

      step = Step.Oscillating;
      timer.restart();
    } else if (progressCheck && !sufficientProgress) {
      Kp *= PIDTuningConstants.KpIncreaseBase + (1 - progress) * PIDTuningConstants.KpIncreaseProportion;
    }
  }

  public void oscillating() {

  }

  public void measuring() {

  }

  public void resetting() {
    
  }

  public double getLoopTarget() {
    // TODO: handle looping systems (i.e. rotating wheels)
    double currentMeasurement = getSystemOutput.get();
    double baselineDistance = Math.abs(currentMeasurement - systemInformation.baseline);
    double targetDistance = Math.abs(currentMeasurement - systemInformation.reasonableTarget * PIDTuningConstants.overshootAdjustRatio);

    if (baselineDistance < targetDistance) {
      return systemInformation.baseline;
    } else {
      return systemInformation.reasonableTarget * PIDTuningConstants.overshootAdjustRatio;
    }
  }
}
