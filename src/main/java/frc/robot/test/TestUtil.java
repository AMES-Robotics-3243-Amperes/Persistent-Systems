// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.concurrent.Future;
import java.util.function.Supplier;

/** Add your docs here. */
public class TestUtil {

    /**
     * A method for asking the user questions as part of integrated testing. This
     * is something you might want to do because you can't be sure encoders are always
     * acurate - those need to be checked too.
     * 
     * @param question  The question to ask the user
     * @param optionYes The text displayed on the button returning true
     * @param optionNo  The text displayed on the button returning false
     * @return          A {@link Future} which will eventually contain the user's decision.
     *                  True means the "yes" option was picked. False means the "No" option was picked.
     *                  Null means the user closed the window or it ended in another manner.
     * 
     * @author H!
     */
    public static Future<Boolean> askUserBool(String question, String optionYes, String optionNo) {
        return TestManager.driverStationClient.askQuestion(question, optionYes, optionNo);
    }

    /**An overload where {@code optionYes} and {@code optionNo} are presumed to be "Yes" and "No"
     * @see TestUtil#askUserBool(String, String, String)
     */
    public static Future<Boolean> askUserBool(String question) {
        return askUserBool(question, "Yes", "No");
    }

    /**
     * Asserts that one number is equal to another, and if this is false, throws an error,
     * typically to be caught by a test manager, or to stop the program.
     * 
     * @param a The first number
     * @param b The second number
     * @param message The optional error message
     * @param error The allowable error
     * @throws AssertionError
     */
    public static void assertEquals(double a, double b, String message, double error) throws AssertionError {
        if (Math.abs(a-b) > error) {
            throw new AssertionError(message);
        }
    }

    /**
     * Asserts that one number is equal to another, and if this is false, throws an error,
     * typically to be caught by a test manager, or to stop the program.
     * 
     * Defaulting to an error of no more than one part in a million.
     * 
     * @param a The first number
     * @param b The second number
     * @param message The optional error message
     * @throws AssertionError
     */
    public static void assertEquals(double a, double b, String message) throws AssertionError {
        assertEquals(a, b, message, ((a+b)/2.)*1E-6);
    }

    /**
     * Asserts that one number is equal to another, and if this is false, throws an error,
     * typically to be caught by a test manager, or to stop the program.
     * 
     * Defaulting to an error of no more than one part in a million, and to have a simple error message.
     * 
     * @param a The first number
     * @param b The second number
     * @throws AssertionError
     */
    public static void assertEquals(double a, double b) throws AssertionError {
        assertEquals(a, b, a+" was not equal to "+b, ((a+b)/2.)*1E-6);
    }

    /**
     * Asserts that one number is equal to another, and if this is false, throws an error,
     * typically to be caught by a test manager, or to stop the program.
     * 
     * Defaulting to have a simple error message.
     * 
     * @param a The first number
     * @param b The second number
     * @param message The optional error message
     * @throws AssertionError
     */
    public static void assertEquals(double a, double b, double error) throws AssertionError {
        assertEquals(a, b, a+" was not equal to "+b, error);
    }


    /**
     * Asserts that the boolean should be true, and throws an error if it is not.
     * @param shouldBeTrue A boolean to test
     * @param message The error message to pair with the {@link AssertionError}
     * 
     * @author H!
     */
    public static void assertBool(boolean shouldBeTrue, String message) {
        if (!shouldBeTrue) {
            throw new AssertionError(message);
        }
    }

    /**
     * Asserts that the boolean should be true, and throws an error if it is not. The message given is "Assertion failed".
     * @param shouldBeTrue A boolean to test
     * 
     * @author H!
     */
    public static void assertBool(boolean shouldBeTrue) {  assertBool(shouldBeTrue, "Assertion failed");  }


    /** A test that runs once and ends immediately. Intended to be created from a preexisting function. */
    public static class InstantTest implements Test {

        protected Runnable execute;
        protected String name;
        protected Test[] dependencies;
        protected boolean[] successRequirements;

        /**
         * Creates an InstantTest.
         * @param execute The function to run.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be ommitted)
         */
        public InstantTest(Runnable execute, String name, Test[] dependencies, boolean[] successRequirements) {
            this.execute = execute;
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
        }

        /**
         * Creates an InstantTest. This overload assummes all dependencies are required to succeed.
         * @param execute The function to run.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         */
        public InstantTest(Runnable execute, String name, Test[] dependencies) {
            this(execute, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates an InstantTest. This overload assumes there are no dependencies.
         * @param execute The function to run.
         * @param name The name of the test.
         */
        public InstantTest(Runnable execute, String name) {
            this(execute, name, new Test[0], new boolean[0]);
        }

        protected static boolean[] generateBoolArray(Test[] list) {
            boolean[] out = new boolean[list.length];
            for (int i = 0; i < out.length; i++) {
                out[i] = true;
            }
            return out;
        }



        @Override
        public void periodic() { execute.run(); }
        @Override
        public boolean isDone() { return true; }
        @Override
        public String getName() { return name; }
        @Override
        public Test[] getDependencies() { return dependencies; }
        @Override
        public boolean[] getDependencySuccessRequirements() { return successRequirements; }
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.METHOD)
    @Inherited
    @interface InstantTestMethod {
        String name() default "";
    }


    /** A test that runs until a specified condition is met. Intended to be created from two existing functions. */
    public static class OnePhaseTest implements Test {

        protected Runnable periodicFunc;
        protected String name;
        protected Test[] dependencies;
        protected boolean[] successRequirements;
        protected Supplier<Boolean> isDoneFunc;

        /**
         * Creates a OnePhaseTest.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be ommitted)
         */
        public OnePhaseTest(Runnable periodic, Supplier<Boolean> isDone, String name, Test[] dependencies, boolean[] successRequirements) {
            this.periodicFunc = periodic;
            this.isDoneFunc = isDone;
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
        }

        /**
         * Creates a OnePhaseTest. This overload assummes all dependencies are required to succeed.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         */
        public OnePhaseTest(Runnable execute, Supplier<Boolean> isDone, String name, Test[] dependencies) {
            this(execute, isDone, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates a OnePhaseTest. This overload assumes there are no dependencies.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         */
        public OnePhaseTest(Runnable execute, Supplier<Boolean> isDone, String name) {
            this(execute, isDone, name, new Test[0], new boolean[0]);
        }

        protected static boolean[] generateBoolArray(Test[] list) {
            boolean[] out = new boolean[list.length];
            for (int i = 0; i < out.length; i++) {
                out[i] = true;
            }
            return out;
        }



        @Override
        public void periodic() { periodicFunc.run(); }
        @Override
        public boolean isDone() { return isDoneFunc.get(); }
        @Override
        public String getName() { return name; }
        @Override
        public Test[] getDependencies() { return dependencies; }
        @Override
        public boolean[] getDependencySuccessRequirements() { return successRequirements; }
    }



     /** A test that runs until a specified condition is met. Intended to be created from two existing functions. */
    public static class MultiphaseTest implements Test {

        protected Runnable[] phases;
        protected String name;
        protected Test[] dependencies;
        protected boolean[] successRequirements;
        protected Supplier<Boolean>[] phaseEndConditions;
        protected int phase = 0;
        protected final int phaseCount;

        /**
         * Creates a MultiphaseTest.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be ommitted)
         */
        public MultiphaseTest(Runnable[] phases, Supplier<Boolean>[] phaseEndConditions, String name, Test[] dependencies, boolean[] successRequirements) {
            this.phases = phases;
            this.phaseEndConditions = phaseEndConditions;
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
            phaseCount = phases.length;
            
            if (phaseEndConditions.length != phaseCount) {
                throw new IllegalArgumentException("Number of phases must equal number of phase end conditions");
            }
        }

        /**
         * Creates a MultiphaseTest. This overload assummes all dependencies are required to succeed.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be ommitted)
         */
        public MultiphaseTest(Runnable[] phases, Supplier<Boolean>[] phaseEndConditions, String name, Test[] dependencies) {
            this(phases, phaseEndConditions, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates a MultiphaseTest. This overload assumes there are no dependencies.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         */
        public MultiphaseTest(Runnable[] phases, Supplier<Boolean>[] phaseEndConditions, String name) {
            this(phases, phaseEndConditions, name, new Test[0], new boolean[0]);
        }

        protected static boolean[] generateBoolArray(Test[] list) {
            boolean[] out = new boolean[list.length];
            for (int i = 0; i < out.length; i++) {
                out[i] = true;
            }
            return out;
        }

        @Override
        public void setup() {
            phase = 0;
        }

        @Override
        public void periodic() { 
            phases[phase].run();
            if (phaseEndConditions[phase].get()) {
                phase++;
            }
        }

        @Override
        public boolean isDone() { 
            return phase == phaseCount;
        }



        @Override
        public String getName() { return name; }
        @Override
        public Test[] getDependencies() { return dependencies; }
        @Override
        public boolean[] getDependencySuccessRequirements() { return successRequirements; }
    }

}
