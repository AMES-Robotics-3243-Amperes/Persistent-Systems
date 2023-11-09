// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.plaf.DimensionUIResource;
import javax.swing.text.html.HTMLDocument;

import frc.robot.Robot;

/** A system which will run all tests queued to it and display their results. @author H! */
public class TestManager {

    /**
     * Stores the results of a test, that being whether it succeeded and any other message
     * it may have provided.
     * 
     * @author H!
     */
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

    /**
     * Used for storing the progression of a test.
     * 
     * @author H!
     */
    protected static enum TestState {
        setup,
        running,
        closedown
    }



    public static Map<String, Map<String, TestResults>> results = new HashMap<String, Map<String, TestResults>>();
    protected static List<SubsystemBaseTestable> subsystemsToTest = new ArrayList<SubsystemBaseTestable>();

    protected static int testIndex = 0;
    protected static TestState testState = TestState.setup;

    public static boolean testsFinished = false;






    /**
     * Used to add a subsystem to be tested to the queue. This is an external access point.
     * 
     * @param toTest The {@link SubsystemBaseTestable} to test
     * 
     * @author H!
     */
    public static void queueSubsystemToTest(SubsystemBaseTestable toTest) {
        subsystemsToTest.add(toTest);
    }

    /**
     * Should be run when test mode is started by {@link Robot#testInit()}. Resets everything and clears the test queue.
     * 
     * @author H!
     */
    public static void init() {
        subsystemsToTest.clear();
        testIndex = 0;
        testState = TestState.setup;
        testsFinished = false;
    }

    /**
     * Should be run periodically by {@link Robot#testPeriodic()}. Runs queued tests.
     * 
     * @author H!
     */
    public static void periodic() {
        if (subsystemsToTest.size() > 0) {
            results.putIfAbsent(subsystemsToTest.get(0).getName(), new HashMap<String, TestResults>());
            runTests(subsystemsToTest.get(0));
        } else {
            if (!testsFinished) {
                displayTestResults();
            }

            testsFinished = true;
        }
    }

    /**
     * Runs one cycle of the current test on the given subsystem. Should be run periodically.
     * When all tests are done, removes this element from {@link #subsystemsToTest} and resets {@link #testIndex}
     * 
     * @param subsystem The subsystem to run the tests of
     * 
     * @author H!
     */
    protected static void runTests(SubsystemBaseTestable subsystem) {
        if (testIndex >= subsystem.tests.length) {
            subsystemsToTest.remove(subsystem);
            testIndex = 0;
            return;
        }

        runTest(subsystem.tests[testIndex]);
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

    /** The format used to generate the HTML subystem headers in the test results @author H! */
    protected static final String subsystemFormat = "<li><h2 class='%2$s'>%1$s | %3$s</h2> <p><em class='success'>%4$d/%6$d Succeess</em> | <em class='fail'>%5$d/%6$d Fails</em></p><ul></ul></li>";
    /**A utility method for getting the proper HTML to make a subsytem test result wrapper be displayed
     * 
     * @param resultEntry One entry of the map corresponding to the subsystem to display
     * @return A {@link String} with the HTML in plaintext repersenting the subystem header
     * 
     * @author H!
     */
    protected static String getSubsystemHTMLElement(Entry<String, Map<String, TestResults>> resultEntry) {
        int successCount = 0;
        int totalCount = resultEntry.getValue().size();

        for (TestResults testResult : resultEntry.getValue().values()) {
            if (testResult.m_didSucceed) {
                successCount++;
            }
        }

        return String.format(
            subsystemFormat, 
            resultEntry.getKey(), 
            successCount == totalCount ? "success" : "fail",
            successCount == totalCount ? "✔" : "✗",
            successCount,
            totalCount - successCount,
            totalCount
        );
    }

    /** The format used to generate the HTML for individual tests in the test results @author H! */
    protected static final String testFormat = "<li><h3 class='%2$s'>%1$s | %3$s</h3>%4$s</li>";
    /**A utility method for getting the proper HTML to make a test result be displayed
     * 
     * @param testEntry One entry of the map corresponding to the test to display
     * @return A {@link String} with the HTML in plaintext repersenting the test result
     * 
     * @author H!
     */
    protected static String getTestHTMLFormat(Entry<String, TestResults> testEntry) {
        return String.format(
            testFormat, 
            testEntry.getKey(), 
            testEntry.getValue().m_didSucceed ? "success" : "fail",
            testEntry.getValue().m_didSucceed ? "✔" : "✗",
            testEntry.getValue().m_message.equals("") ? "" : "<p>" + testEntry.getValue().m_message + "</p>"
        );
    }

    /**Displays the latest results of the integrated tests in a Swing dialog
     * @author H!
     */
    public static void displayTestResults() {
        JTextPane textPane = new JTextPane();
        textPane.setContentType("text/html");
        textPane.setText("<!DOCTYPE html><html><head><style>.fail{color:red}.success{color:green}</style></head><body><h1>Integrated Test Results</h1><ul id=subsystemList></ul></body></html>");
        HTMLDocument doc = (HTMLDocument) textPane.getDocument();

        for (Entry<String, Map<String, TestResults>> subsystemEntry : results.entrySet()) {
            try {
                doc.insertAfterStart(doc.getElement("subsystemList"), getSubsystemHTMLElement(subsystemEntry));

                for (Entry<String, TestResults> testResultEntry : subsystemEntry.getValue().entrySet()) {
                    doc.insertAfterStart(doc.getElement("subsystemList").getElement(0).getElement(2), getTestHTMLFormat(testResultEntry));
                }


            } catch (Exception e) {
                System.out.println("Test result display generation failed:");
                e.printStackTrace();
            }
        }
        
        JScrollPane scrollPane = new JScrollPane(textPane);

        
        scrollPane.setPreferredSize(new DimensionUIResource(
            600, 
            500
        ));
        
        JOptionPane.showMessageDialog(
            new JTextPane(), 
            scrollPane,
            "Integrated Test Results",
            JOptionPane.PLAIN_MESSAGE
        );
    }
}
