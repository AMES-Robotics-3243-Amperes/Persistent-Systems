import static org.junit.jupiter.api.Assertions.assertEquals;


import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;

/** <h3>An example test using WPI's unit testing system.</h3>
 * 
 * <p>See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html for detail</p>
 * 
 * <p>Use "Ctrl + Shift + P => Test Robot Code" to run all unit tests</p>
 * 
 * <h4>Recommended test configuration:</h4>
 * <ul>
 *   <li>Skip tests on deploy - (Ctrl + Shift + P => Change Skip Tests On Deploy Setting => Yes (project))</li>
 *   <li>Ensure build.gradle contains this:
 * test {
 *  useJUnitPlatform()
 *  systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
 * }</li>
 * </ul>
 * 
 * <p>Note that WPILib can often behave in odd ways. You may have to comment and uncomment the relevant
 * portion of the build.gradle for the imports to work</p>
 * 
 * <p>Look for the file:///C:... link in the log output for a nice display for the tests</p>
 */
public class ExampleTest {

    /** Acceptable range of floating point error */
    protected static final double DELTA = 1e-2;
    /** Some example object for our tests to do stuff on */
    protected Translation3d trans3d;
    
    @BeforeEach
    public void setup() {
        trans3d = new Translation3d();
    }

    @Test
    public void nothingTest() {
        assertEquals(2, 1+1);
    }

    @Test
    public void addition() {
        Translation3d newTrans3d = trans3d.plus(new Translation3d(3, 0.23, -0.977));
        assertEquals(3,     newTrans3d.getX(), DELTA, "Expected: 3  Actual: " + trans3d.getX());
        assertEquals(0.23,  newTrans3d.getY(), DELTA, "Expected: 0.23  Actual: " + trans3d.getY());
        assertEquals(-0.977,newTrans3d.getZ(), DELTA,  "Expected: -0.977  Actual: " + trans3d.getZ());
    }

    

    @AfterEach
    public void shutdown() {
        // If needed, call your subsystem's .close method here
    }
}

