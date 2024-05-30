package frc.robot;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DataManagerConstants;
import frc.robot.Constants.DriveTrainConstants.ChassisKinematics;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
    /** :3 Singleton instance */
    private static DataManager instance;

    /** :3 used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private Photonvision photonvision;
    private IMU imu = new AHRS_IMU();

    /**
     * Constructs the singleton from member variables of a RobotContainer.
     * Should be called ASAP, preferably in the constructor for RobotContainer.
     * 
     * @author :3
     */
    public DataManager(RobotContainer robotContainer) {
        subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
        photonvision = robotContainer.photonvision;

        poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics,
            imu.getRotation(),
            subsystemSwerveDrivetrain.getModulePositions(),
            new Pose2d());

        instance = this;
    }

    /**
     * Updates all important data in DataManager. Should be called
     * periodically *after* the command loop has concluded.
     * 
     * @author :3
     */
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
    }

    /**
     * @author :3
     * @return the latest robot pose
     */
    public static Pose2d getRobotPose() {
        return instance.poseEstimator.getEstimatedPosition();
    }
}
