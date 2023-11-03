// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public interface Test {
    public void testPeriodic();

    public boolean testIsDone();

    public void setupPeriodic();

    public boolean setupIsDone();

    public void closedownPeriodic();

    public boolean closedownIsDone();

    public String getName();
}
