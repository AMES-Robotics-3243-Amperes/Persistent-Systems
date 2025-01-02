package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonvisionConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.PhotonUnit.Measurement;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
  /** Singleton instance */
  private static DataManager instance;

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
        Optional<Measurement> measurementOptional = unit.getMeasurement();

        if (measurementOptional.isPresent()) {
          Measurement measurement = measurementOptional.get();
          poseEstimator.addVisionMeasurement(measurement.pose, measurement.timestampSeconds,
              measurement.ambiguity.times(PhotonvisionConstants.poseEstimatorAmbiguityScaleFactor
                  * poseEstimator.getEstimatedPosition().getTranslation().getDistance(measurement.targetPosition)));
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
