// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utility.FeedforwardControllerSimpleMotor;
import frc.robot.utility.PIDFMotorController;

/** Add your docs here. */
public class SwerveModule {
    CANSparkMax propulsionMotor;
    CANSparkMax turningMotor;

    AbsoluteEncoder turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    double turnOffset;
    SwerveModuleState targetState = new SwerveModuleState();

    SimpleMotorFeedforward propulsionFeedforward;
    FeedforwardControllerSimpleMotor turningFeedforward;
    PIDController turningPID;
    PIDFMotorController turningPIDF;


    public SwerveModule(CANSparkMax propulsionMotor, CANSparkMax turningMotor, double turnOffset) {
        this.propulsionMotor = propulsionMotor;
        this.turningMotor = turningMotor;
        this.turnOffset = turnOffset;

        propulsionFeedforward = new SimpleMotorFeedforward(0,0,0);
        turningFeedforward = new FeedforwardControllerSimpleMotor(0,0,0);

        turningPIDF = new PIDFMotorController(turningMotor, turningPID, turningFeedforward, turningEncoder);

    }

    public void setState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, Rotation2d.fromRotations(turningEncoder.getPosition()));
        

        propulsionMotor.setVoltage(propulsionFeedforward.calculate(targetState.speedMetersPerSecond));
        turningPIDF.setSetpoint(targetState.angle.getRotations());
    }



    public void periodic() {
        turningPIDF.update();
    }


    static class ModuleState {
        double angle;
        double speed;
    
        public ModuleState(double angle, double speed) {
            this.angle = angle;
            this.speed = speed;
        }

        public double angleOffset(double offset) {
            return angle + offset;
        }
    }
}
