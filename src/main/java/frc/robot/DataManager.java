package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveTrainConstants.ChassisKinematics;
import frc.robot.subsystems.SubsystemPhotonvision;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;

public class DataManager {
    /** :3 Singleton instance */
    private static DataManager instance;

    /** :3 the latest calculated robot position */
    private Pose2d robotPose;

    /** :3 used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private SubsystemPhotonvision subsystemPhotonvision;
    private IMU imu = new AHRS_IMU();

    /**
     * Constructs the singleton from member variables of a RobotContainer.
     * Should be called ASAP, preferably in the constructor for RobotContainer.
     * 
     * @author :3
     */
    public DataManager(RobotContainer robotContainer) {
        subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
        subsystemPhotonvision = robotContainer.subsystemPhotonvision;

        poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics,
            imu.getRotationModulus(),
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
        throw new UnsupportedOperationException("DataManager.update() not implemented");
    }

    /**
     * @author :3
     * @return the latest robot pose
     */
    public static Pose2d getRobotPose() {
        return instance.robotPose;
    }
}
