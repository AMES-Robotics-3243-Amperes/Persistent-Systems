// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class SubsystemBaseTestable extends SubsystemBase {

    public Test[] tests;

    protected boolean isTesting = false;

    @Override
    public void periodic() {
        if (DriverStation.isTest()) {
            if (!isTesting) {
                onTestStart();
            }
            isTesting = true;
        } else {
            isTesting = false;
        }
    }

    public void onTestStart() {
        TestManager.queueSubsystemToTest(this);
    }
}
