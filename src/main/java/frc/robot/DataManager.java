package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DifferentialArm.Setpoints;
import frc.robot.Constants.PhotonvisionConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
// import frc.robot.Constants.Setpoints;
import frc.robot.Constants.Setpoints.LevelAngles;
import frc.robot.Constants.Setpoints.LevelHeights;
import frc.robot.PhotonUnit.Measurement;
import frc.robot.subsystems.SubsystemElevator;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
  /** Singleton instance */
  private static DataManager instance;

  // PROPOSAL: Deprecate or remove this method.
  // REASON: Maintaining Dependency Inversion (the D in SOLID) to avoid
  //   DataManager being used before it is configured, or used in places
  //   another system would be better. The DataManager instance should always
  //   be accessed through dependency injection (passing a thing into a constructor)
  //   to this end.
  // H!
  public static DataManager instance() {
    return instance;
  }

  /**
   * An {@link Entry} that is automatically updated upon a call to
   * {@link #update DataManager.instance().update}.
   */
  public abstract class DataManagerEntry<T> extends Entry<T> {
    public DataManagerEntry() {
      entries.add(this);
    }
  }

  public class RobotPosition extends DataManagerEntry<Pose2d> {
    /** used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private List<PhotonUnit> photonUnits = new ArrayList<PhotonUnit>();
    private IMU imu = new AHRS_IMU();

    private Field2d field2d = new Field2d();

    public RobotPosition(RobotContainer robotContainer) {
      subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
      this.photonUnits = new ArrayList<PhotonUnit>(PhotonvisionConstants.photonUnits);

      poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics, imu.getRotation(),
          subsystemSwerveDrivetrain.getModulePositions(), new Pose2d());
    }

    public void update() {
      poseEstimator.update(imu.getRotation(), subsystemSwerveDrivetrain.getModulePositions());

      for (var unit : photonUnits) {
        for (Measurement measurement : unit.getMeasurement()) {
          poseEstimator.addVisionMeasurement(measurement.pose, measurement.timestampSeconds,
              measurement.ambiguity.times(PhotonvisionConstants.poseEstimatorAmbiguityScaleFactor
                  * (poseEstimator.getEstimatedPosition().getTranslation().getDistance(measurement.targetPosition)
                      + 1)));
        }
      }

      field2d.setRobotPose(get());
      SmartDashboard.putData(field2d);
    }

    public void set(Pose2d newPose) {
      poseEstimator.resetPosition(imu.getRotationModulus(), subsystemSwerveDrivetrain.getModulePositions(), newPose);
    }

    public Pose2d get() {
      return poseEstimator.getEstimatedPosition();
    }
  }

  // H! TODO Put constants in Constants.java
  public static enum ElevatorSetpoint {
    Between(null),
    L1(Constants.Elevator.Positions.L1),
    L2(Constants.Elevator.Positions.L2),
    L3(Constants.Elevator.Positions.L3),
    L4(Constants.Elevator.Positions.L4);

    public final Double position;

    private ElevatorSetpoint(Double position) {
      this.position = position;
    }
  }

  public static class ElevatorPositionData {
    // H! TODO Put constants in Constants.java
    public static final double DELTAP = 0.15;
    public static final double DELTAV = 0.05;

    public final double exactPos;
    public final double exactVel;
    public final ElevatorSetpoint setpoint;

    public ElevatorPositionData(double position, double velocity) {
      this.exactPos = position;
      this.exactVel = velocity;

      ElevatorSetpoint generalPosToBeSet = ElevatorSetpoint.Between;
      double minPositionDif = DELTAP; // Position dif must always be than DELTAP
      for (ElevatorSetpoint value : ElevatorSetpoint.values()) {
        if (value.position == null) {continue;} // Skip over ElevatorSetpoint.Between
        if (Math.abs(velocity) < DELTAV && Math.abs(value.position - position) < minPositionDif) {
          generalPosToBeSet = value;
          minPositionDif = Math.abs(value.position - position);
          // Continue in case a future setpoint is closer.
        }
      }

      setpoint = generalPosToBeSet;
    }
  }

  public class ElevatorPosition extends DataManagerEntry<ElevatorPositionData> {
    private SubsystemElevator elevator;
    public ElevatorPosition(RobotContainer robotContainer) {
      elevator = robotContainer.subsystemElevator;
    }

    @Override
    public ElevatorPositionData get() {
      return new ElevatorPositionData(elevator.getPosition(), elevator.getVelocity());
    }
  }

  // public enum SetpointDiffArm {
  //   Starting(Setpoints.startingPosition, Setpoints.startingPower),
  //   Intake(Setpoints.intakePosition, Setpoints.intakePower),
  //   PlaceL1(Setpoints.depositPositionL1, Setpoints.depositPower);

  //   public final double position;
  //   public final double power;

  //   private SetpointDiffArm(double position, double power) {
  //     this.position = position;
  //     this.power = power;
  //   }
  // }

  public enum Setpoint {
    Intake(LevelHeights.Intake, LevelAngles.Intake),
    L1(LevelHeights.L1, LevelAngles.L1),
    L2(LevelHeights.L2, LevelAngles.L23),
    L3(LevelHeights.L3, LevelAngles.L23),
    L4(LevelHeights.L4, LevelAngles.L4);

    public final double height;
    public final double angle;

    private Setpoint(double height, double angle) {
      this.height = height;
      this.angle = angle;
    }
  }

  @SuppressWarnings("rawtypes")
  private ArrayList<DataManagerEntry> entries = new ArrayList<>();

  public DataManagerEntry<Pose2d> robotPosition;

  public void update() {
    entries.forEach(entry -> entry.update());
  }

  /**
   * Constructs the singleton from member variables of a RobotContainer.
   * Should be called ASAP, preferably in the constructor for RobotContainer.
   */
  public DataManager(RobotContainer robotContainer) {
    robotPosition = new RobotPosition(robotContainer);

    instance = this;
  }
}
