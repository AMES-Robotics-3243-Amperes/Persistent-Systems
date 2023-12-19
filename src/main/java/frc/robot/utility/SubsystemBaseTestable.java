// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Extend this class to have it automatically run tests in the integrated testing framework managed by {@link TestManager} @author H! */
public class SubsystemBaseTestable extends SubsystemBase implements TestGroup {

    /** An array of test objects to be run when test mode is activated. Fill this on contruction or class initialization. @author H! */
    public Test[] tests;

    @Override
    public Test[] getTests() {
        return tests;
    }


    protected boolean isTesting = false;

    
    /** <h3>CANNOT BE EXTENDED</h3> <p>This is to force certain logic to always be used. 
     * Use {@link SubsystemBaseTestable#doPeriodic()} for things that must run periodically.</p>
     * {@inheritDoc}
     */
    @Override
    public final void periodic() {
        if (DriverStation.isTest()) {
            if (!isTesting) {
                onTestStart();
            }
            isTesting = true;
        } else {
            isTesting = false;
        }

        doPeriodic();
    }

    /** 
     * Used instead of periodic to ensure certain logic always runs periodically. 
     * Override this instead of {@link #periodic()} @author H!
     */
    public void doPeriodic() {}


    public void onTestStart() {
        TestManager.queueGroupToTest(this);
    }
}
