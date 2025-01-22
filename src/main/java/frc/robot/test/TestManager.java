// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import frc.robot.Robot;
import frc.robot.test.TestUtil.InstantTest;
import frc.robot.test.TestUtil.InstantTestMethod;
import frc.robot.test.networking.Workstation;

/** A system which will run all tests queued to it and display their results. @author H! */
public class TestManager {
    /* TODO:
     * Make the test runner behave nicely if the testing is cut off halfway through
     * Other Utils
     * 
     * Paralalizable (<- spelled wrong):
     */

    public enum TestSuccess {
        NOTRUN,
        FAIL,
        SUCCESS;
    }

    public TestSuccess testSuccessFromBool(Boolean bool) {
        if (bool == true) {
            return TestSuccess.SUCCESS;
        } else if (bool == false) {
            return TestSuccess.FAIL;
        } else {
            return TestSuccess.NOTRUN;
        }
    }

    /**
     * Stores the results of a test, that being whether it succeeded and any other message
     * it may have provided.
     * 
     * @author H!
     */
    public static class TestResults {
        public TestSuccess m_successResult;
        public String m_message;

        public TestResults(TestSuccess successResult, String message) {
            m_successResult = successResult;
            m_message = message;
        }

        public TestResults(TestSuccess successResult) {
            this(successResult, "");
        }
    }


    public static Map<String, Map<String, TestResults>> results = new HashMap<String, Map<String, TestResults>>();
    protected static Map<Test, TestSuccess> testsRun = new HashMap<Test, TestSuccess>();

    protected static List<TestGroup> groupsToTest = new ArrayList<TestGroup>();
    protected static List<Test> testsToTest = new ArrayList<Test>();


    protected static int testIndex = 0;

    public static boolean testsFinished = false;
    public static boolean testStarted = false;
    private static int initialPauseLength = 5;
    private static int initialPauseTimer = 0;
    protected static boolean testSelectionMade = false;

    protected static int cyclesRun = 0;

    protected static Workstation driverStationClient;
    private static Future<boolean[]> selectedTestGroups;






    /**
     * Used to add a test group to be tested to the queue. This is an external access point.
     * 
     * @param toTest The {@link TestGroup} to test
     * 
     * @author H!
     */
    public static void queueGroupToTest(TestGroup toTest) {
        groupsToTest.add(toTest);
    }

    protected static Test[] getTestsFromGroup(TestGroup group) {
        // WARNING: terribly cursed reflection, keep out
        List<Test> annotatedTests = new ArrayList<Test>();
        try {
            for (Method method : group.getClass().getMethods()) {
                if (method.isAnnotationPresent(InstantTestMethod.class)) {
                    InstantTestMethod testAnnotation = method.getAnnotation(InstantTestMethod.class);
                    annotatedTests.add(new InstantTest(
                        () -> {
                            try {
                                method.invoke(group);
                            } catch (InvocationTargetException e) {
                                if (e.getCause() instanceof AssertionError) {
                                    throw (AssertionError) e.getCause();
                                } else {
                                    throw new RuntimeException(e.toString());
                                }
                            } catch (IllegalAccessException | IllegalArgumentException e) {
                                throw new RuntimeException(e.toString());
                            }
                        }, 
                        testAnnotation.name().length() > 0 ? testAnnotation.name() : method.getName()
                    ));
                }
            }
        } catch (IllegalArgumentException | SecurityException e) {
            throw new RuntimeException(e.toString());
        }
        // This part is fine
        return addListToArray(group.getTests(), annotatedTests);
    }

    /**
     * Should be run when test mode is started by {@link Robot#testInit()}. Resets everything and clears the test queue.
     * 
     * @author H!
     */
    public static void init() {
        groupsToTest.clear();
        testsToTest.clear();
        testIndex = 0;
        initialPauseTimer = initialPauseLength;
        testsFinished = false;
        selectedTestGroups = new CompletableFuture<>();
        testSelectionMade = false;
        cyclesRun = 0;
        results = new HashMap<String, Map<String, TestResults>>();
        testsRun = new HashMap<Test, TestSuccess>();
    }

    public static void onDisable() {
        driverStationClient.close();
        driverStationClient = new Workstation();
    }

    /**
     * Should be run as soon as possible, and only once. Configures some things such
     * as starting the TCP server. These won't happen until the first method call otherwise.
     */
    public static void load() {
        driverStationClient = new Workstation();
    }

    /**
     * Should be run periodically by {@link Robot#testPeriodic()}. Runs queued tests.
     * 
     * @author H!
     */
    public static void periodic() {
        if (initialPauseTimer > 0) {
            initialPauseTimer--;
            return;
        } else if (initialPauseTimer == 0) {
            showTestGroupSelection();
            initialPauseTimer--;
            return;
        }

        if (!testSelectionMade) {
            // Check if decision was made, if it was, apply the changes and move on.
            if (selectedTestGroups.isDone()) {
                System.out.println("Test group selection received");
                try {
                    boolean[] selectedGroupsUnwrapped = selectedTestGroups.get();
                    // Remove those test groups not selected
                    int numberRemoved = 0;
                    for (int i = 0; i < selectedGroupsUnwrapped.length; i++) {
                        if (!selectedGroupsUnwrapped[i]) {
                            groupsToTest.remove(i - numberRemoved);
                            numberRemoved++;
                        }
                    }
                } catch (InterruptedException | ExecutionException e) {
                    throw new RuntimeException(e);
                }

                testSelectionMade = true;
            }
            return;
        }

        cyclesRun++;
        String[] testGroupNames = new String[groupsToTest.size()];
        for (int i = 0; i < groupsToTest.size(); i++) {
            testGroupNames[i] = groupsToTest.get(i).getName();
        }

        if (groupsToTest.size() > 0) {
            results.putIfAbsent(groupsToTest.get(0).getName(), new HashMap<String, TestResults>());
            runTests(groupsToTest.get(0));

        } else {
            if (!testsFinished) {
                displayTestResults();;
                testsFinished = true;
            }
        }
    }

    /**
     * Runs one cycle of the current test on the given test group. Should be run periodically.
     * When all tests are done, removes this element from {@link #groupsToTest} and resets {@link #testIndex}
     * 
     * @param testGroup The test group to run the tests of
     * 
     * @author H!
     */
    protected static void runTests(TestGroup testGroup) {
        if (testsToTest.size() == 0) {
            testsToTest = new ArrayList<Test>(Arrays.asList(getTestsFromGroup(testGroup)));
        }

        if (!testStarted) {
            /* DEPENDENCY LOGIC:
             * 
             * All done, all correct         -> Run test
             * All done, not all correct     -> Mark test as not run, and remove it from the queue
             * Not all done, all correct     -> Buffer with dependencies, move test to the back of the queue
             * Not all done, not all correct -> Mark test as not run, and remove it from the queue
             * 
             * A dependency being "correct" means that its result (success/failure) matched the result the main test required of the dependency.
             */
            // Check if dependencies are already done:
            boolean allDependenciesDone = true;
            boolean allDependenciesCorrect = true;
            for (int i = 0; i < testsToTest.get(0).getDependencies().length; i++) {
                Test test = testsToTest.get(0).getDependencies()[i];
                TestSuccess isRun = testsRun.get(test);
                if (isRun == null) {
                    allDependenciesDone = false;
                } else if (isRun == TestSuccess.FAIL && testsToTest.get(0).getDependencySuccessRequirements()[i] == true) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.SUCCESS && testsToTest.get(0).getDependencySuccessRequirements()[i] == false) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.NOTRUN) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                }
            }

            if (!allDependenciesCorrect) {
                results.get(groupsToTest.get(0).getName()).put(testsToTest.get(0).getName(), new TestResults(TestSuccess.NOTRUN, "Dependencies Not Correct"));
                testsRun.put(testsToTest.remove(0), TestSuccess.NOTRUN);
                if (testsToTest.size() == 0) {
                    groupsToTest.remove(0);
                }
                testStarted = false;
                return;
            } else if (!allDependenciesDone) {
                for (Test test : testsToTest.get(0).getDependencies()) {
                    if (!testsRun.containsKey(test) && !testsToTest.contains(test)) {
                        testsToTest.add(1, test);
                    }
                }

                testsToTest.add(testsToTest.remove(0));
                return; // Return to this method next cycle now that the test list has been updated
            }
            testsToTest.get(0).setup();
        }
        runTest(testsToTest.get(0));
    }

    /** Runs all logic that must run when a tests finishes.
     * This involves managing resetting counters and preparing the next tests.
     */
    public static void onTestDone(Test test) {
        test.closedown();
        testsToTest.remove(0);
        if (testsToTest.size() == 0) {
            groupsToTest.remove(0);
        }
        testStarted = false;
    }

    /**
     * Logic to run one cycle of a test. Should be run periodically to perform the test.
     * When done, increments to the next test.
     * 
     * @param test The test to run
     * 
     * @author H!
     */
    protected static void runTest(Test test) {
        testStarted = true;
        try {
            test.periodic();
            if (test.isDone()) {
                results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.SUCCESS));
                testsRun.put(test, TestSuccess.SUCCESS);
                onTestDone(test);
            }
        } catch (AssertionError e) {
            results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.FAIL, e.getMessage()));
            testsRun.put(test, TestSuccess.FAIL);
            onTestDone(test);
        }
    }

    /**Displays the latest results of the integrated tests in a Swing dialog
     * @author H!
     */
    public static void displayTestResults() {
        driverStationClient.publishResults(results);
    }


    public static void showTestGroupSelection() {
        // Incoming data
        String[] testGroupNames = new String[groupsToTest.size()];
        for (int i = 0; i < groupsToTest.size(); i++) {
            testGroupNames[i] = groupsToTest.get(i).getName();
        }

        selectedTestGroups = driverStationClient.getChosenTestGroups(testGroupNames);
    }



    private static <T> T[] addListToArray(T[] array, List<T> list) {
        T[] out = Arrays.copyOf(array, array.length + list.size());
        Iterator<T> iterator = list.iterator();
        for (int i = array.length; i < out.length; i++) {
            out[i] = iterator.next();
        }

        return out;
    }
}
