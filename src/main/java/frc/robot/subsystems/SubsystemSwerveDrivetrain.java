package frc.robot.subsystems;

import java.util.concurrent.Future;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.SwerveConstants.DriveTrainConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.test.SubsystemBaseTestable;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;

public class SubsystemSwerveDrivetrain extends SubsystemBaseTestable {
  private final SubsystemSwerveModule m_frontLeft = new SubsystemSwerveModule(
      DriveTrainConstants.IDs.kFrontLeftDrivingCanId,
      DriveTrainConstants.IDs.kFrontLeftTurningCanId, DriveTrainConstants.ModuleOffsets.kFrontLeftOffset);

  private final SubsystemSwerveModule m_frontRight = new SubsystemSwerveModule(
      DriveTrainConstants.IDs.kFrontRightDrivingCanId,
      DriveTrainConstants.IDs.kFrontRightTurningCanId, DriveTrainConstants.ModuleOffsets.kFrontRightOffset);

  private final SubsystemSwerveModule m_rearLeft = new SubsystemSwerveModule(
      DriveTrainConstants.IDs.kRearLeftDrivingCanId,
      DriveTrainConstants.IDs.kRearLeftTurningCanId, DriveTrainConstants.ModuleOffsets.kBackLeftOffset);

  private final SubsystemSwerveModule m_rearRight = new SubsystemSwerveModule(
      DriveTrainConstants.IDs.kRearRightDrivingCanId,
      DriveTrainConstants.IDs.kRearRightTurningCanId, DriveTrainConstants.ModuleOffsets.kBackRightOffset);

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

  @Override
  public void doPeriodic() {
    m_frontLeft.update();
    m_frontRight.update();
    m_rearLeft.update();
    m_rearRight.update();
  }

  public SysIdRoutine driveRoutine = new SysIdRoutine(
      new Config(BaseUnits.Voltage.of(0.75).per(BaseUnits.Time.of(1)),
          BaseUnits.Voltage.of(3),
          BaseUnits.Time.of(8),
          null),
      new Mechanism(
          this::sysIdDrive,
          this::sysIdDriveLog,
          this));

  /**
   * Sets the modules' drive voltages to a specific value
   * 
   * @param voltage the value to set the drive voltages to
   */
  public void sysIdDrive(Measure<Voltage> voltage) {
    m_frontLeft.driveVoltage(voltage.baseUnitMagnitude());
    m_frontRight.driveVoltage(voltage.baseUnitMagnitude());
    m_rearLeft.driveVoltage(voltage.baseUnitMagnitude());
    m_rearRight.driveVoltage(voltage.baseUnitMagnitude());
  }

  /**
   * Logs the motors' info. For use with SysID.
   * 
   * @param log The {@link SysIdRoutineLog} to log to
   */
  public void sysIdDriveLog(SysIdRoutineLog log) {
    m_frontLeft.driveLog(log.motor("front_left_drive"));
    m_frontRight.driveLog(log.motor("front_right_drive"));
    m_rearLeft.driveLog(log.motor("rear_left_drive"));
    m_rearRight.driveLog(log.motor("rear_right_drive"));
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

  // #################
  // INTEGRATION TESTS
  // #################

  @Override
  public Test[] getTests() {
    return tests;
  }

  private boolean moduleRotationTest2Done = false;
  private Future<Boolean> moduleRotationTestUserQuestion;
  private Test[] tests = {
    new TestUtil.MultiphaseTest(
      new Runnable[] {this::moduleRotationTest1, this::moduleRotationTest2, this::moduleRotationTest3},
      (Supplier<Boolean>[]) new Supplier[] {() -> true, () -> moduleRotationTest2Done, () -> true}, 
      "Module Rotation Test"
    )
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
