// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/** Add your docs here. */
public class TestManager {

    public static class TestResults {
        public boolean m_didSucceed;
        public String m_message;

        public TestResults(boolean didSucceed, String message) {
            m_didSucceed = didSucceed;
            m_message = message;
        }

        public TestResults(boolean didSucceed) {
            this(didSucceed, "");
        }
    }

    public static Map<String, Map<String, TestResults>> results = new HashMap<String, Map<String, TestResults>>();

    protected static enum TestState {
        setup,
        running,
        closedown
    }

    protected static List<SubsystemBaseTestable> subsystemsToTest = new ArrayList<SubsystemBaseTestable>();

    public static void queueSubsystemToTest(SubsystemBaseTestable toTest) {
        subsystemsToTest.add(toTest);
    }

    public static void init() {
        subsystemsToTest.clear();
        testIndex = 0;
        testState = TestState.setup;
    }

    public static void periodic() {
        if (subsystemsToTest.size() > 0) {
            results.putIfAbsent(subsystemsToTest.get(0).getName(), new HashMap<String, TestResults>());
            runTests(subsystemsToTest.get(0));
        }
    }

    protected static int testIndex = 0;

    protected static void runTests(SubsystemBaseTestable subsystem) {
        if (testIndex >= subsystem.tests.length) {
            subsystemsToTest.remove(0);
            return;
        }

        runTest(subsystem.tests[testIndex]);
    }

    protected static TestState testState = TestState.setup;

    protected static void runTest(Test test) {
        try {
            switch (testState) {
                case setup:
                    test.setupPeriodic();
                    if (test.setupIsDone()) {
                        testState = TestState.running;
                    }
                    break;
            
                case running:
                    test.testPeriodic();
                    if (test.testIsDone()) {
                        testState = TestState.closedown;
                    }
                    break;
                
                case closedown:
                    test.closedownPeriodic();
                    if (test.closedownIsDone()) {
                        testState = TestState.setup;
                        testIndex++;
                        results.get(subsystemsToTest.get(0).getName()).put(test.getName(), new TestResults(true));
                    }
                    break;
                
                default:
                    break;
            }
        } catch (AssertionError e) {
            results.get(subsystemsToTest.get(0).getName()).put(test.getName(), new TestResults(false, e.getMessage()));
        }
    }
}
