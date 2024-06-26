package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonVision;
import frc.robot.Constants.DriveTrain.DriveConstants.ChassisKinematics;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.PowerManager;

/** <b>Stores all of the data that is shared between systems, especially positions.</b>
 * 
 * <p>Note that the entries in this class should be responsible for getting the correct information 
 * at the correct time. The classes from which this takes information should never interact 
 * directly with classes which use their information. Most processing of the information provided
 * by systems should be done in this class.</p>
 * 
 * <p>Also, it's possible you'll have to write entries that don't fit the {@link Entry} interface;
 * this is ok.</p>
 * 
 * @implNote This class has method calls in {@link Robot}
 * 
 * @author H!
 */
public class DataManager {
    //#########################################################
    //                    ENTRY FRAMEWORK
    //#########################################################


    /** Stores some kind of data of type {@link T} to be interchanged between systems
     *  @author H!
     */
    public static interface Entry<T> {
        /** Returns the data stored in this entry
         *  @return The data stored in this entry
         */
        T get();
    }


    /** An {@link Entry} that can be set using a set method
     * @author H!
     */
    public static interface SettableEntry<T> extends Entry<T> {
        /**
         * Sets the value of this entry
         * @param newValue The new value for the entry
         */
        void set(T newValue);
    }

    /** An entry that does not change throughout time
     * @author H!
     */
    public static class StaticEntry<T> implements Entry<T> {
        protected final T m_value;
        /** 
         * 
         * @param value
         */
        public StaticEntry(T value) {
            m_value = value;
        }

        @Override
        public T get() {
            return m_value;
        }
    }


    /** An entry for a position and orientation in 3D @author H! */
    public static interface FieldPose extends Entry<Pose3d> {}
    /** An entry for a position in 3D @author H! */
    public static interface FieldLocation extends Entry<Translation3d> {}


    //#########################################################
    //                      ENTRY TYPES
    //#########################################################
    

    /** The entry for the position and orientation of the robot
     * 
     */
    public static class CurrentRobotPose implements FieldPose {
        /** Does the heavy lifting for keeping track of Pose @author :3 */
        protected SwerveDrivePoseEstimator m_PoseEstimator;

        protected Field2d field2d = new Field2d();

        /**
         * Creates a new {@link CurrentRobotPose} object
         */
        public CurrentRobotPose() {
            // TODO add anything that is needed here
        }

        /**
         * Reconstructs the {@link SwerveDrivePoseEstimator} with the correct information
         * Used since the static construction of the class makes certain things difficult in the main constructor
         * 
         * @author :3
         */
        public void constructPoseEstimator(SubsystemSwerveDrivetrain drivetrainSubsystem) {
            // creates a pose estimator with provided information
            m_PoseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics, drivetrainSubsystem.getIMUHeading(),
                drivetrainSubsystem.getModulePositions(), new Pose2d());
        }

        @Override
        public Pose3d get() {
            if (m_PoseEstimator == null) {
                return new Pose3d();
            }

            return new Pose3d(m_PoseEstimator.getEstimatedPosition());
        }

        public void updateSmartDashboard() {
            field2d.setRobotPose(this.get().toPose2d());
            SmartDashboard.putData(field2d);
        }

        /**
         * Updates the robot's position using odometry.
         * Should be run periodically.
         * 
         * @param odometryReading the current Pose2d reported by odometry
         */
        public void updateWithOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
            // TODO: consider imu position data
            if (m_PoseEstimator == null) {
                return;
            }
            
            m_PoseEstimator.update(gyroAngle, modulePositions);
        }

        public void updateWithVision(Pose3d visionEstimate, double timestampSeconds, double ambiguity) {
            double distrust = PhotonVision.visionDistrust * ambiguity;
            m_PoseEstimator.addVisionMeasurement(visionEstimate.toPose2d(), timestampSeconds, VecBuilder.fill(distrust, distrust, distrust));
        }
        
    }

    public static class AccelerationConstant implements Entry<Double> {
        public Double get() {
            return (PowerManager.getDriveAccelerationDampener());
        }
    }

    public static class VelocityConstant implements Entry<Double> {
        public Double get() {
            return (PowerManager.getDriveSpeedDamper());
        }
    }

    //#########################################################
    //                        ENTRIES
    //#########################################################

    public static CurrentRobotPose currentRobotPose = new CurrentRobotPose();
    public static AccelerationConstant currentAccelerationConstant = new AccelerationConstant();
    public static VelocityConstant currentVelocityConstant = new VelocityConstant();

    //#########################################################
    //               INITIALIZATION AND RUNTIME
    //#########################################################

    /**Called after robot container is done being constructed. This happens
     * when the robot code is deployed.
     * 
     * Put all construction of entries here, instead of in the definition. This
     * ensures we have control over when those entries are constructed. You may want
     * to put some further initialization in the following methods, called at different times.
     */
    public static void onRobotInit() {
        currentRobotPose = new CurrentRobotPose();
    }

    /** Called when the robot is enabled. */
    public static void onEnable() {}

    /** Called when the robot is disabled, from {@link Robot#disabledInit()} */
    public static void onDisable() {}

    /** Called when the robot is enabled in teleop, from {@link Robot#teleopInit()} */
    public static void onTeleopInit() {
        onEnable();
    }

    /** Called when the robot is enabled in autonomous, from {@link Robot#autonomousInit()} */
    public static void onAutoInit() {
        onEnable();
    }

    /** Called when the robot is enabled in test mode, from {@link Robot#testInit()} */
    public static void onTestInit() {
        onEnable();
    }

    /** Called when the robot is created during a simulation, from {@link Robot#simulationInit()} */
    public static void onSimulationInit() {}

    /**Called every 20 milliseconds, from {@link Robot#robotPeriodic()}. Try to avoid using this method
     * if you can, there's a decent chance if you're using it this is really something that should
     * be done by a subsystem or command.
     */
    public static void periodic() {
        currentRobotPose.updateSmartDashboard();
    }
}
