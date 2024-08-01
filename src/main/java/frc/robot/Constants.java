// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final double kRateLimitLeft = 5;
    public static final double kRateLimitRight = 5;
    public static final double exponent1 = 3;
    public static final double exponent2 = 1;
    public static final double coeff1 = 0;
    public static final double coeff2 = 1;

    public static final double leftTriggerSpeedMultiplier = 1.6;
    public static final double rightTriggerSpeedMultiplier = 0.4;
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
        public static final int kFrontLeftDrivingCanId = 8;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 4;
        public static final int kRearRightDrivingCanId = 6;

        public static final int kFrontLeftTurningCanId = 7;
        public static final int kRearLeftTurningCanId = 1;
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

  public static final class PhotonvisionConstants {
    public static final String cameraName = "Global_Shutter_Camera (1)";

    public static final Pose3d cameraPosition =
      new Pose3d(new Translation3d(Units.inchesToMeters(10.5),
        Units.inchesToMeters(0),
        Units.inchesToMeters(13)),
        new Rotation3d(0, Math.PI / 4, 0));
    public static final Transform3d robotToCamera = new Transform3d(new Pose3d(), cameraPosition);
  }

  public static final class DataManagerConstants {
    /**
     * Takes photon ambiguity and turns it into pose estimator ambiguity @author :3
     */
    public static final double photonPoseEstimatorAmbiguity(Double photonAmbiguity) {
      return 5 * photonAmbiguity * photonAmbiguity;
    }
  }

  public static final class CurveConstants {
    /**
     * As the robot drifts from the spline, the speed at
     * which the setpoint travels across the spline decreases.
     * This function determines the speed multiplier as a function
     * of robot offset from the spline. @author :3
     */
    public static final double splineOffsetVelocityDampen(double offset) {
      return 1 / (1 + 1.5 * offset * offset);
    }

    public static final double halfLoopTime = 0.01;
    public static final int newtonIterations = 4;
    public static final double maxCentrifugalAcceleration = 2.2;
  }
}
