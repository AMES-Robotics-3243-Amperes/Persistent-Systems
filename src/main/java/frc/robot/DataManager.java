package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.PhotonUnit.Measurement;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
  /** :3 Singleton instance */
  private static DataManager instance;

  public static DataManager instance() {
    return instance;
  }

  public abstract class DataManagerEntry<T> extends Entry<T> {
    public DataManagerEntry() {
      entries.add(this);
    }
  }

  /**
   * A {@link DataManager} entry that is automatically updated upon a call to
   * {@link #update DataManager.instance().update}.
   */
  public class RobotPosition extends DataManagerEntry<Pose2d> {
    /** :3 used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private ArrayList<PhotonUnit> photonUnits = new ArrayList<PhotonUnit>();
    private IMU imu = new AHRS_IMU();

    private Field2d field2d = new Field2d();

    public RobotPosition(RobotContainer robotContainer) {
      // TODO: move photon stuff to constants
      subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;

      AprilTag tag = new AprilTag(1, new Pose3d(new Translation3d(15, 0, 0), new Rotation3d(0, 0, Math.PI)));
      ArrayList<AprilTag> tags = new ArrayList<>();
      tags.add(tag);
      var fieldLayout = new AprilTagFieldLayout(tags, 20, 20);

      photonUnits.add(new PhotonUnit("FrontCamera", fieldLayout));

      poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics,
        imu.getRotation(),
        subsystemSwerveDrivetrain.getModulePositions(),
        new Pose2d());
    }

    public void update() {
      poseEstimator.update(imu.getRotation(), subsystemSwerveDrivetrain.getModulePositions());

      for (var unit : photonUnits) {
        Optional<Measurement> measurementOptional = unit.getMeasurement();
        
        if (measurementOptional.isPresent()) {
          Measurement measurement = measurementOptional.get();
          poseEstimator.addVisionMeasurement(measurement.pose, measurement.timestampSeconds, measurement.ambiguity);
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
   * 
   * @author :3
   */
  public DataManager(RobotContainer robotContainer) {
    robotPosition = new RobotPosition(robotContainer);

    instance = this;
  }
}
