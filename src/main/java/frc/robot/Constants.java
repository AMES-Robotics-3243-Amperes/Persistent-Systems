// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class JoyUtilConstants {
    // :3 size of controller deadzone
    public static final double kDeadzone = 0.12;

    // :3 max amount controller output can change per second
    public static final double kRateLimitLeft = 20;
    public static final double kRateLimitRight = 20;

    // :3 curve stuff
    public static final int exponent1 = 1;
    public static final int exponent2 = 3;
    public static final double coeff1 = 0.4;
    public static final double coeff2 = 0.6;

    // :3 fast and slow mode
    public static final double leftTriggerSpeedMultiplier = 1.3;
    public static final double rightTriggerSpeedMultiplier = 0.55;

    // :3 ports
    public static int primaryControllerID = 0;
    public static int secondaryControllerID = 1;
  }

  public static final class DriveTrain {

    // :3 constants for individual modules
    public static final class ModuleConstants {
      // :3 pid connects at 0 and 2 pi because rotation is continuous
      public static final double kTurningEncoderPositionPIDMinInput = 0; // :3 radians
      public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2; // :3 radians

      // :3 idle modes
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      // :3 current limits
      public static final int kDrivingMotorCurrentLimit = 50; // :3 amps
      public static final int kTurningMotorCurrentLimit = 20; // :3 amps

      // :3 pidf values / min and max outputs
      public static final class PIDF {

        public static final double kDrivingP = 0.4;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.45;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
      }

      // :3 everything having to do with the physical assembly of the modules
      public static final class PhysicalProperties {

        /**
         * :3 direct quote from rev robotics:
         * <p>
         * The MAXSwerve module can be configured with one of three pinion gears:
         * 12T, 13T, or 14T. This changes the drive speed of the module
         * (a pinion gear with more teeth will result in a robot that drives faster).
         */
        public static final int kDrivingMotorPinionTeeth = 13;

        // :3 if constructed correctly, all modules' turning encoders will be reversed
        public static final boolean kTurningEncoderInverted = true;

        // :3 required for various calculations
        public static final double kWheelDiameterMeters = 0.0762;
      }

      // all the encoder factors
      public static final class EncoderFactors {

        // :3 quote from rev robotics:
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction =
          (45.0 * 22) / (PhysicalProperties.kDrivingMotorPinionTeeth * 15);

        public static final double kDrivingEncoderPositionFactor =
          (PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor =
          ((PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
      }
    }

    public static final class DriveConstants {
      // :3 speed damper (flat constant supplied speed is multiplied by)
      public static final double kDrivingSpeedDamper = 1; // :3 meters per second
      public static final double kAngularSpeedDamper = 0.5 * Math.PI; // :3 radians per second

      // :3 the max physical speed of the modules
      // :3 THIS IS NOT THE MAX DRIVING SPEED
      public static final double kMaxObtainableModuleSpeed = 10;

      // :3 global rate limits
      // (generally, use controller and pid rate limiting
      // instead of this, this is mainly here to keep the transitions
      // between commands and/or input styles smooth)
      public static final double kMaxRotationAcceleration = 1.5 * Math.PI;
      public static final double kMaxDrivingAcceleration = 0.2;

      // :3 if the gyro is reversed
      public static final boolean kGyroReversed = false;

      // :3 spark max ids
      public static final class IDs {

        // :3 driving ids
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 9;
        public static final int kFrontRightDrivingCanId = 5;
        public static final int kRearRightDrivingCanId = 13;

        // :3 turning ids
        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 15;
      }

      // :3 absolute encoder offsets (should be multiples of pi / 2
      // :3 if the encoders were zeroed properly in rev client)
      public static final class ModuleOffsets {
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(Math.PI * 0.5);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(0);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(Math.PI * 1.5);
      }

      // :3 things involving the physical setup of the chassis
      public static final class ChassisKinematics {

        // :3 distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(27);
        // :3 distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(32);

        // :3 kinematics (defined with above constants)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2), new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
      }

      // :3 max temperatures for the drive train motors
      public static final class TempConstants {
        public static final double max550TempCelsius = 100;
        public static final double max1650TempCelsius = 100;
      }

      // :3 auto-movement configuration
      public static final class AutoConstants {
        // :3 turning stuff
        public static final double kMaxAngularVelocityRadians = 3;
        public static final double kMaxAngularAccelerationRadians = 2.5;

        public static final double kTuringP = 0.1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;

        public static final TrapezoidProfile.Constraints kTurningConfig =
          new TrapezoidProfile.Constraints(kMaxAngularVelocityRadians, kMaxAngularAccelerationRadians);

        public static final ProfiledPIDController kTurningPID =
          new ProfiledPIDController(kTuringP, kTurningI, kTurningD, kTurningConfig);
      }
    }
  }
}
