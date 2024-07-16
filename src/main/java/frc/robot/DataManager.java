package frc.robot;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DataManagerConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
  /** :3 Singleton instance */
  private static DataManager instance;

  public static DataManager instance() {
    return instance;
  }

  public interface Entry<T> {
    /** Updates the entry, called in DataManager.update() */
    public default void update() {}

    /** Gets the entry. Called from DataManager.instance.entry.get() */
    public abstract T get();
  }

  public class RobotPosition implements Entry<Pose2d> {
    /** :3 used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private Photonvision photonvision;
    private IMU imu = new AHRS_IMU();

    private Field2d field2d = new Field2d();

    public RobotPosition(RobotContainer robotContainer) {
      subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
      photonvision = robotContainer.photonvision;

      poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics,
        imu.getRotation(),
        subsystemSwerveDrivetrain.getModulePositions(),
        new Pose2d());
    }

    public void update() {
      poseEstimator.update(imu.getRotation(), subsystemSwerveDrivetrain.getModulePositions());

      var poseLatestOptional = photonvision.getPhotonPose();
      if (poseLatestOptional.isPresent()) {
        EstimatedRobotPose poseLatest = poseLatestOptional.get().getFirst();
        double distrust = DataManagerConstants.photonPoseEstimatorAmbiguity(poseLatestOptional.get().getSecond());

        poseEstimator.addVisionMeasurement(poseLatest.estimatedPose.toPose2d(),
          poseLatest.timestampSeconds,
          VecBuilder.fill(distrust, distrust, distrust));
      }

      field2d.setRobotPose(get());
      SmartDashboard.putData(field2d);
    }

    public Pose2d get() {
      return poseEstimator.getEstimatedPosition();
    }
  }

  public final Entry<Pose2d> robotPosition;

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

  /**
   * Updates all DataManager entries. Should be called after commands/subsystems
   * have
   * ran their periodic functions to avoid race conditions.
   */
  public void update() {
    robotPosition.update();
  }
}
