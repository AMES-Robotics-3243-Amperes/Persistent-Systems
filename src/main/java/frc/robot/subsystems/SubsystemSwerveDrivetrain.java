package frc.robot.subsystems;

import java.util.concurrent.Future;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SwerveConstants.DriveTrainConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.modules.SwerveModule;
import frc.robot.subsystems.modules.ThriftyModule;
import frc.robot.test.SubsystemBaseTestable;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;

public class SubsystemSwerveDrivetrain extends SubsystemBaseTestable {
  private final SwerveModule m_frontLeft = new ThriftyModule(
      DriveTrainConstants.IDs.kFrontLeftDrivingCanId,
      DriveTrainConstants.IDs.kFrontLeftTurningCanId, 0, DriveTrainConstants.ModuleOffsets.kFrontLeftOffset);

  private final SwerveModule m_frontRight = new ThriftyModule(
      DriveTrainConstants.IDs.kFrontRightDrivingCanId,
      DriveTrainConstants.IDs.kFrontRightTurningCanId, 1, DriveTrainConstants.ModuleOffsets.kFrontRightOffset);

  private final SwerveModule m_rearLeft = new ThriftyModule(
      DriveTrainConstants.IDs.kRearLeftDrivingCanId,
      DriveTrainConstants.IDs.kRearLeftTurningCanId, 2, DriveTrainConstants.ModuleOffsets.kBackLeftOffset);

  private final SwerveModule m_rearRight = new ThriftyModule(
      DriveTrainConstants.IDs.kRearRightDrivingCanId,
      DriveTrainConstants.IDs.kRearRightTurningCanId, 3, DriveTrainConstants.ModuleOffsets.kBackRightOffset);

  public SubsystemSwerveDrivetrain() {
  }

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxObtainableModuleSpeed);

    // :3 set the desired states
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Set the swerve modules' desired rotations. Does not optimize rotations.
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
   * 
   * @author :3
   */
  public void setModuleRotations(Rotation2d[] desiredRotations) {
    // :3 set the desired states
    m_frontLeft.setDesiredRotation(desiredRotations[0]);
    m_frontRight.setDesiredRotation(desiredRotations[1]);
    m_rearLeft.setDesiredRotation(desiredRotations[2]);
    m_rearRight.setDesiredRotation(desiredRotations[3]);
  }

  /**
   * Used for pose estimation.
   * 
   * @author :3
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_rearLeft.getPosition(), m_rearRight.getPosition()
    };
  }

  /**
   * Used for pose estimation.
   * 
   * @author :3
   * @return the positions of the swerve modules
   */
  public SwerveModulePosition[] getAbsoluteModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getAbsolutePosition(), m_frontRight.getAbsolutePosition(),
        m_rearLeft.getAbsolutePosition(), m_rearRight.getAbsolutePosition()
    };
  }

  @Override
  public void doPeriodic() {
    m_frontLeft.update();
    m_frontRight.update();
    m_rearLeft.update();
    m_rearRight.update();
  }

  // #####
  // SYSID
  // #####

  private final SysIdRoutine driveRoutine = new SysIdRoutine(
      new Config(BaseUnits.VoltageUnit.of(0.9).per(Units.Second),
          BaseUnits.VoltageUnit.of(2.4),
          BaseUnits.TimeUnit.of(4.5),
          (state) -> SignalLogger.writeString("state", state.toString())),
      new Mechanism(
          this::sysIdDrive,
          null,
          this));

  /**
   * Sets the modules' drive voltxages to a specific value
   * 
   * @param voltage the value to set the drive voltages to
   */
  public void sysIdDrive(Voltage voltage) {
    m_frontLeft.driveVoltage(voltage.in(Units.Volts));
    m_frontRight.driveVoltage(voltage.in(Units.Volts));
    m_rearLeft.driveVoltage(voltage.in(Units.Volts));
    m_rearRight.driveVoltage(voltage.in(Units.Volts));
  }

  /**
   * @param direction The direction for a quasistatic routine to run
   * @return A quasistatic routine
   */
  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction);
  }

  /**
   * @param direction The direction for a dynamic routine to run
   * @return A dynamic routine
   */
  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }

  /**
   * Sets the SysID state. This affects the behavior of the modules,
   * enabling SysID will remove some functionality that conflicts
   * with SysID.
   */
  public void setSysID(boolean doingSysID) {
    m_frontLeft.setSysID(doingSysID);
    m_frontRight.setSysID(doingSysID);
    m_rearLeft.setSysID(doingSysID);
    m_rearRight.setSysID(doingSysID);
  }

  // #################
  // INTEGRATION TESTS
  // #################

  @Override
  public Test[] getTests() {
    return tests;
  }

  private boolean moduleRotationTest2Done = false;
  private Future<Boolean> moduleRotationTestUserQuestion;
  @SuppressWarnings("unchecked")
  private Test[] tests = {
      new TestUtil.MultiphaseTest(
          new Runnable[] { this::moduleRotationTest1, this::moduleRotationTest2, this::moduleRotationTest3 },
          (Supplier<Boolean>[]) new Supplier[] { () -> true, () -> moduleRotationTest2Done, () -> true },
          "Module Rotation Test")
  };

  /**
   * Ensures the {@link setModuleRotations} method functions properly, according
   * to user viewing, and encoders.
   * 
   * Part 1.
   */
  private void moduleRotationTest1() {
    setModuleRotations(new Rotation2d[] {
        new Rotation2d(0),
        new Rotation2d(0),
        new Rotation2d(0),
        new Rotation2d(0)
    });

    moduleRotationTest2Done = false;
    moduleRotationTestUserQuestion = TestUtil.askUserBool("Are all wheels pointing forward?");
  }

  private void moduleRotationTest2() {
    if (moduleRotationTestUserQuestion.isDone()) {
      boolean areWheelsCorrect;
      try {
        areWheelsCorrect = moduleRotationTestUserQuestion.get();
      } catch (Exception e) {
        throw new AssertionError(e);
      }

      TestUtil.assertBool(areWheelsCorrect, "Wheels did not point forward when given that command.");

      moduleRotationTest2Done = true;
    }
  }

  private void moduleRotationTest3() {
    SwerveModulePosition[] positions = getModulePositions();

    for (SwerveModulePosition position : positions) {
      double angleDif = position.angle.minus(new Rotation2d()).getDegrees() % 360;
      if (angleDif > 10 || angleDif < -10) {
        throw new AssertionError("Swerve module encoder did not show forward facing direction.");
      }
    }
  }
}
