// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final class JoyUtilConstants {
        public static final double kDeadzone = 0.05;
        public static final double kRateLimitLeft = 0;
        public static final double kRateLimitRight = 0;
        public static final double exponent1 = 2;
        public static final double exponent2 = 1;
        public static final double coeff1 = 0.4;
        public static final double coeff2 = 0.6;
        
        public static final double leftTriggerSpeedMultiplier = 1.5;
        public static final double rightTriggerSpeedMultiplier = 0.75;
    }

    public static final class DriveTrainConstants {
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
    }
}
