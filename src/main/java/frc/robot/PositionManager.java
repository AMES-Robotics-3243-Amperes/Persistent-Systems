package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Stores all of the data that is shared between systems, especially positions
 * 
 * Note that the entries in this class should be responsible for getting the correct information 
 * at the correct time. The classes from which this takes information should never interact 
 * directly with classes which use their information. Most processing of the information provided
 * by systems should be done in this class.
 * 
 * @apiNote This class has method calls in {@link Robot}
 * 
 * @author H!
 */
public class PositionManager {

    //#########################################################
    //                    ENTRY FRAMEWORK
    //#########################################################


    /** Stores some kind of data of type {@link T} to be interchanged between systems
     *  @author H!
     */
    public static interface Entry<T> {
        /** Returns the data stored in this entry
         *  @return The data stored in this entry
         */
        T get();
    }


    /** An {@link Entry} that can be set using a set method
     * @author H!
     */
    public static interface SettableEntry<T> extends Entry<T> {
        /**
         * Sets the value of this entry
         * @param newValue The new value for the entry
         */
        void set(T newValue);
    }

    /** An entry that does not change throughout time
     * 
     */
    public static class StaticEntry<T> implements Entry<T> {
        protected final T m_value;
        /** 
         * 
         * @param value
         */
        public StaticEntry(T value) {
            m_value = value;
        }

        @Override
        public T get() {
            return m_value;
        }
    }


    /** An entry for a position and orientation in 3D @author H! */
    public static interface FieldPose extends Entry<Pose3d> {}
    /** An entry for a position in 3D @author H! */
    public static interface FieldLocation extends Entry<Translation3d> {}


    //#########################################################
    //                      ENTRY TYPES
    //#########################################################
    

    /** The entry for the positionn and orientation of the robot
     * 
     */
    public static class CurrentRobotPose implements FieldPose {
        /** Creates a new {@link CurrentRobotPose} object
         * 
         */
        public CurrentRobotPose() {
            // TODO add anything that is needed here
        }

        @Override
        public Pose3d get() {
            // TODO Auto-generated method stub
            return null;
        }
        
    }

    //#########################################################
    //                        ENTRIES
    //#########################################################

    public static CurrentRobotPose currentRobotPose;

    //#########################################################
    //                     INITIALIZATION
    //#########################################################

    /**Called after robot container is done being constructed.
     * 
     * Put all construction of entries here, instead of in the definition. This
     * ensures we have control over when those entries aare constructed. You may want
     * to put some further initialization in the folowing methods, called at different times.
     */
    public static void onSystemConstruction() {
        currentRobotPose = new CurrentRobotPose();
    }
}
