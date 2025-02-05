// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import javax.sound.sampled.AudioFormat.Encoding;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.splines.interpolation.CubicInterpolator;
import frc.robot.splines.interpolation.SplineInterpolator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class JoyUtilConstants {
    public static final double kDeadzone = 0.05;
    public static final double kRateLimitLeft = 20;
    public static final double kRateLimitRight = 20;
    public static final double exponent1 = 3;
    public static final double exponent2 = 1;
    public static final double coeff1 = 0.2;
    public static final double coeff2 = 0.8;

    public static final double leftTriggerSpeedMultiplier = 2.0;
    public static final double rightTriggerSpeedMultiplier = 0.3;
  }

  public static final class SwerveConstants {
    public static final class ControlConstants {
      public static final double movingSpeed = 1.5;
      public static final double rotationSpeed = 1.5 * Math.PI;
    }

    public static final class ChassisKinematics {
      // :3 distance between centers of right and left wheels on robot
      public static final double kRobotWidth = Units.inchesToMeters(15);

      // :3 distance between front and back wheels on robot
      public static final double kRobotLength = Units.inchesToMeters(15);

      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
    }

    public static final class DriveTrainConstants {
      public static final class IDs {
        public static final int kFrontLeftDrivingCanId = 2;
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearRightDrivingCanId = 6;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 7;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kRearRightTurningCanId = 5;
      }

      public static final class ModuleOffsets {
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(-Math.PI * 0.5);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(0);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(Math.PI * 0.5);
      }
    }

    public static final class ModuleConstants {
      // :3 the max physical speed of the modules. NOT drive speed
      // (for now, I really have no clue what this should
      // be, so I have it set unreasonably high)
      public static final double kMaxObtainableModuleSpeed = 100;

      // :3 pid connects at 0 and 2 pi because rotation is continuous
      public static final double kTurningEncoderPositionPIDMinInput = 0;
      public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2;

      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      public static final int kDrivingMotorCurrentLimit = 60;
      public static final int kTurningMotorCurrentLimit = 25;

      public static final class PhysicalProperties {
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final boolean kTurningEncoderInverted = true;
        public static final double kWheelDiameterMeters = 0.0762;
      }

      public static final class EncoderFactors {
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22)
            / (PhysicalProperties.kDrivingMotorPinionTeeth * 15);

        public static final double kDrivingEncoderPositionFactor = (PhysicalProperties.kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor = ((PhysicalProperties.kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0;
        public static final double test = 1 / kDrivingEncoderVelocityFactor / 60;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
      }

      public static final class PIDF {
        public static final double kDrivingP = 0.21485;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;

        public static final double kDrivingKs = 0.10469;
        public static final double kDrivingKv = 2.4783;
        public static final double kDrivingKa = 0.36003;

        public static final double kTurningP = 0.23;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
      }
    }
  }

  public static class Elevator {
    public static class Motors {
      public static final int leaderCanId = 10;
      public static final int followerCanId = 11;

      public static final double positionConversionRatio = 1.0;

      public static final double P = 0.01;
      public static final double I = 0.0;
      public static final double D = 0.0;
      public static final double FF = 0.01;
    }

    public static class PositionChecking {
      public static final double deltaP = 0.05;
      public static final double deltaV = 0.05;
    }

    /** Positions are measured by the pivot position height above minimum. All heights are in meters. */
    public static class Positions {
      /** The vertical distance from the ground to the minimum height on the elevator. */
      private static final double minHeightOffset = Units.inchesToMeters(9.307579);
      public static final double min = 0.0;
      public static final double starting = min;
      public static final double loading = Units.inchesToMeters(28.807579) - minHeightOffset;
      public static final double L1 = Units.inchesToMeters(28.807579) - minHeightOffset; // Not from CAD, but should be similar to loading.
      public static final double L2 = Units.inchesToMeters(37.807579) - minHeightOffset;
      public static final double L3 = Units.inchesToMeters(52.807579) - minHeightOffset;
      public static final double L4 = Units.inchesToMeters(79.807579) - minHeightOffset;
      public static final double max = Units.inchesToMeters(79.807579) - minHeightOffset;
    }
  }

  public static final class PhotonvisionConstants {
    public static final AprilTag tag = new AprilTag(1,
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)));
    public static final List<AprilTag> tags = Arrays.asList(tag);
    public static final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tags, 20, 20);

    public static final List<PhotonUnit> photonUnits = Arrays
        .asList(new PhotonUnit("FrontCamera", PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d(new Pose3d(),
                new Pose3d(new Translation3d(Units.inchesToMeters(9), Units.inchesToMeters(5), Units.inchesToMeters(0)),
                    new Rotation3d(0, Units.degreesToRadians(2), 0))),
            fieldLayout));

    public static final double poseEstimatorAmbiguityScaleFactor = 1.5;
    public static final double photonUnitAmbiguityCutoff = 0.05;
  }

  public static final class SplineConstants {
    public static final class NumericalConstants {
      public static final int compositeGaussianQuadratureIntervals = 3;
      public static final int newtonRaphsonIterations = 10;
    }

    public static final class TaskConstants {
      public static final Rotation2d defaultRotationTolerance = Rotation2d.fromDegrees(8);
      public static final double defaultPositionTolerance = 0.05;
      public static final double defaultPositionBuffer = 0.3;
    }

    public static final class FollowConstants {
      public static final SplineInterpolator defaultInterpolator = new CubicInterpolator();
      public static final double maxSpeed = 4;
      public static final double maxCentrifugalAcceleration = 2;
      public static final double maxAccelAfterTask = 1.5;
      public static final boolean interpolateFromStart = true;

      /**
       * As the robot drifts from the spline, the speed at
       * which the setpoint travels across the spline decreases.
       * This function determines the speed multiplier as a function
       * of robot offset from the spline.
       */
      public static final double splineOffsetVelocityDampen(double offset) {
        return 1 / (1 + 1.5 * offset * offset);
      }

      /**
       * To avoid harsh acceleration, slow the robot's movement as it starts following
       * the path. This function gives a hard velocity cap as a function of length
       * traversed.
       */
      public static final double splineStartVelocityDampen(double length) {
        return 5 * length + 0.2;
      }

      /**
       * To avoid harsh stops, slow the robot's movement as it
       * finishes the path. This function gives a hard velocity
       * cap as a function of remaining length.
       */
      public static final double splineCompleteVelocityDampen(double remainingLength) {
        return 5 * remainingLength + 0.2;
      }

      /**
       * The robot needs to slow down to ensure it executes tasks in the correct
       * position. This command dictates the max velocity of the robot as a function
       * of remaining valid length to execute a task.
       */
      public static final double splineTaskVelocityDampen(double remainingLength) {
        return 5 * remainingLength + 0.2;
      }
    }
  }

  public static final class DifferentialArm {
    // Test this once claw is finished - absolute encoder value at horizontal
    public static final double encoderOffset = 0.510;

    public static final class MotorIDs {
      public static final int leftID = 13;
      public static final int rightID = 14;
    }

    public static final class PID {
      public static final double P = 1.5;
      public static final double I = 0.02;
      public static final double D = 0.06;
    }
  }

  // Important setpoints for the claw
  public static final class Setpoints {
    // Constant intake power (just invert to deposit)
    public static final double intakePower = 0.5;

    // Heights of the different pipe deposit levels, and a height for intake
    public static final class LevelHeights {
      public static final double Intake = 0;
      public static final double Start = 0;
      public static final double L1 = 1;
      public static final double L2 = 2;
      public static final double L3 = 3;
      public static final double L4 = 4;
    }

    // Angles for the different pipe deposit levels, and an angle for intake
    public static final class LevelAngles {
      public static final double Start = 1.5708;
      public static final double Intake = 0.959931;
      public static final double L1 = 0.0;
      public static final double L23 = -0.959931;
      public static final double L4 = -1.5708;
    }
  }
}
