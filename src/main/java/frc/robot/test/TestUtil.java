// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.Optional;
import java.util.concurrent.Future;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class TestUtil {
  
    // Prevent instantiating
    private TestUtil() {}

    /**
     * A method for asking the user questions as part of integrated testing. This
     * is something you might want to do because you can't be sure encoders are always
     * accurate - those need to be checked too.
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
         * @param dependencies The dependencies (can be omitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be omitted)
         */
        public InstantTest(Runnable execute, String name, Test[] dependencies, boolean[] successRequirements) {
            this.execute = execute;
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
        }

        /**
         * Creates an InstantTest. This overload assumes all dependencies are required to succeed.
         * @param execute The function to run.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
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
         * @param dependencies The dependencies (can be omitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be omitted)
         */
        public OnePhaseTest(Runnable periodic, Supplier<Boolean> isDone, String name, Test[] dependencies, boolean[] successRequirements) {
            this.periodicFunc = periodic;
            this.isDoneFunc = isDone;
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
        }

        /**
         * Creates a OnePhaseTest. This overload assumes all dependencies are required to succeed.
         * @param execute The function to run.
         * @param isDone A function returning whether the test is over yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
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



     /** A test with several phases, each of which have an end condition. Intended to be created from several existing functions. */
    public static class MultiphaseTest implements Test {

        protected Runnable[] phases;
        protected String name;
        protected Test[] dependencies;
        protected boolean[] successRequirements;
        protected Supplier<Boolean>[] phaseEndConditions;
        protected int phase = 0;
        protected int phaseCount;

        /**
         * Creates a MultiphaseTest.
         * @param phases An array of the functions to run.
         * @param phaseEndConditions An array of functions specifying whether each phase has ended yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be omitted)
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
         * Creates a MultiphaseTest.
         * @param phases An array of the functions to run.
         * @param phaseEndConditions An array of functions specifying whether each phase has ended yet.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         */
        public MultiphaseTest(Runnable[] phases, Supplier<Boolean>[] phaseEndConditions, String name, Test[] dependencies) {
            this(phases, phaseEndConditions, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates a MultiphaseTest.
         * @param phases An array of the functions to run.
         * @param phaseEndConditions An array of functions specifying whether each phase has ended yet.
         * @param name The name of the test.
         */
        public MultiphaseTest(Runnable[] phases, Supplier<Boolean>[] phaseEndConditions, String name) {
            this(phases, phaseEndConditions, name, new Test[0], new boolean[0]);
        }

        /**
         * Only used for extension, so not everything has to specified immediately.
         */
        private MultiphaseTest() {}

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

    /**
     * A test with multiple phases that execute according to a timer. Created from several methods and timer values 
     */
    public static class TimedTest extends MultiphaseTest {
        protected Timer timer = new Timer();

        /**
         * Creates a TimedTest.
         * @param phases An array of the functions to run.
         * @param durations An array describing how long each phase should last.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be omitted)
         */
        public TimedTest(Runnable[] phases, double[] durations, String name, Test[] dependencies, boolean[] successRequirements) {
            this.phases = phases;
            this.phaseEndConditions = generatePhaseEndConditions(durations);
            this.name = name;
            this.dependencies = dependencies;
            this.successRequirements = successRequirements;
            this.phaseCount = phases.length;
            
            if (phaseEndConditions.length != phaseCount) {
                throw new IllegalArgumentException("Number of phases must equal number of phase end conditions");
            }
        }

        /**
         * Creates a TimedTest.
         * @param phases An array of the functions to run.
         * @param durations An array describing how long each phase should last.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         */
        public TimedTest(Runnable[] phases, double[] durations, String name, Test[] dependencies) {
            this(phases, durations, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates a TimedTest.
         * @param phases An array of the functions to run.
         * @param durations An array describing how long each phase should last.
         * @param name The name of the test.
         */
        public TimedTest(Runnable[] phases, double[] durations, String name) {
            this(phases, durations, name, new Test[0], new boolean[0]);
        }

        @Override
        public void setup() {
            super.setup();
            timer.restart();
        }

        /** A helper method to get the suppliers used by the underlying MultiphaseTest. */
        @SuppressWarnings("unchecked")
        private Supplier<Boolean>[] generatePhaseEndConditions(double[] durations) {
            Supplier<Boolean>[] out = (Supplier<Boolean>[]) new Supplier[durations.length];

            double total = 0.0;
            for (int i = 0; i < durations.length; i++) {
                total += durations[i];
                // Store each closure separately.
                final double timeLimit = total;
                out[i] = () -> timer.hasElapsed(timeLimit);
            }
            return out;
        }
    }

    /**
     * Combines multiple tests into one test which runs each in succession. Similar
     * to SequentialCommandGroup.
     */
    public static class CombinedTest extends MultiphaseTest {
        
        /**
         * Creates a CombinedTest.
         * @param components The test to be joined together in order.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         * @param successRequirements Which dependencies must succeed and which must fail (can be omitted)
         */
        public CombinedTest(Test[] components, String name, Test[] dependencies, boolean[] successRequirements) {
            super(phasesFromTests(components), conditionsFromTests(components), name, dependencies, successRequirements);
        }

        /**
         * Creates a TimedTest.
         * @param phases An array of the functions to run.
         * @param durations An array describing how long each phase should last.
         * @param name The name of the test.
         * @param dependencies The dependencies (can be omitted)
         */
        public CombinedTest(Test[] components, String name, Test[] dependencies) {
            this(components, name, dependencies, generateBoolArray(dependencies));
        }

        /**
         * Creates a TimedTest.
         * @param phases An array of the functions to run.
         * @param durations An array describing how long each phase should last.
         * @param name The name of the test.
         */
        public CombinedTest(Test[] components, String name) {
            this(components, name, new Test[0], new boolean[0]);
        }

        private static Runnable[] phasesFromTests(Test[] components) {
            Runnable[] out = new Runnable[components.length * 3];

            for (int i = 0; i < components.length; i++) {
                out[i*3]   = components[i]::setup;
                out[i*3+1] = components[i]::periodic;
                out[i*3+2] = components[i]::closedown;
            }

            return out;
        }

        private static Supplier<Boolean>[] conditionsFromTests(Test[] components) {
            @SuppressWarnings("unchecked")
            Supplier<Boolean>[] out = (Supplier<Boolean>[]) new Supplier[components.length * 3];

            for (int i = 0; i < components.length; i++) {
                out[i*3]   = () -> true;
                out[i*3+1] = components[i]::isDone;
                out[i*3+2] = () -> true;
            }

            return out;
        }
    }

    /**
     * <h2>Warning, untested code!</h2>
     * Runs a test alongside a command. The test has access to the command, and can run
     * checks during and after. The test logic is always called just before the command logic
     * within a cycle.
     */
    public static class CommandTest<C extends Command> extends OnePhaseTest {

        private C command;
        private Consumer<C> startup;
        //private Consumer<C> during; // Held with closures instead, so unneeded
        //private Consumer<C> ending;

        public CommandTest(C command, Consumer<C> startup, Consumer<C> during, Consumer<C> ending, String name, Test[] dependencies, boolean[] successRequirements) {
            super(
                () -> {
                    if (command.isFinished()) {
                        ending.accept(command);
                    } else {
                        during.accept(command);
                    }
                }, 
                () -> command.isFinished(), 
                name, 
                dependencies, 
                successRequirements
            );
            this.command = command;
            this.startup = startup;
        }

        public CommandTest(C command, Consumer<C> startup, Consumer<C> during, Consumer<C> ending, String name, Test[] dependencies) {
            super(
                () -> {
                    if (command.isFinished()) {
                        ending.accept(command);
                    } else {
                        during.accept(command);
                    }
                }, 
                () -> command.isFinished(), 
                name, 
                dependencies
            );
            this.command = command;
            this.startup = startup;
        }

        public CommandTest(C command, Consumer<C> startup, Consumer<C> during, Consumer<C> ending, String name) {
            super(
                () -> {
                    if (command.isFinished()) {
                        ending.accept(command);
                    } else {
                        during.accept(command);
                    }
                }, 
                () -> command.isFinished(), 
                name
            );
            this.command = command;
            this.startup = startup;
        }

        public CommandTest(C command, Optional<Consumer<C>> startup, Optional<Consumer<C>> during, Optional<Consumer<C>> ending, String name, Test[] dependencies, boolean[] successRequirements) {
            this(command, startup.orElse((c) -> {}), during.orElse((c) -> {}), ending.orElse((c) -> {}), name, dependencies, successRequirements);
        }

        public CommandTest(C command, Optional<Consumer<C>> startup, Optional<Consumer<C>> during, Optional<Consumer<C>> ending, String name, Test[] dependencies) {
            this(command, startup.orElse((c) -> {}), during.orElse((c) -> {}), ending.orElse((c) -> {}), name, dependencies);
        }

        public CommandTest(C command, Optional<Consumer<C>> startup, Optional<Consumer<C>> during, Optional<Consumer<C>> ending, String name) {
            this(command, startup.orElse((c) -> {}), during.orElse((c) -> {}), ending.orElse((c) -> {}), name);
        }

        @Override
        public void setup() {
            command.schedule();
            startup.accept(command);
        }

        @Override
        public void closedown() {
            command.cancel();
        }
    }

}
