/**
 * @author Christian Piper (@CAP1Sup)
 * @since 10/21/21
 */
package frc.robot.enums;

public enum JOYSTICK_OUTPUT_TYPES {

    // Main definition of types (each should have their own index)
    LINEAR(0),
    ZEROED_LINEAR(1),
    ZEROED_QUAD_LINEAR(2);

    // The index of each output type
    private int index;

    /**
     * Creates a new output type
     * @param index The index to set each output type to
     */
    private JOYSTICK_OUTPUT_TYPES(int index) {
        this.index = index;
    }

    /**
     * Gets the integer value for the specified output type
     * @return int index of output type
     */
    public int getInt() {
        return this.index;
    }

    /**
     * Looks up an index and returns the corresponding output type
     * @param index The index that you wish to look up
     * @return Output type matching index
     */
    public static JOYSTICK_OUTPUT_TYPES fromInt(int index) {

        // Loop through all of the possible output types
        for (JOYSTICK_OUTPUT_TYPES type : values()) {

            // Return the type if the index matches
            if (type.getInt() == index) {
                return type;
            }
        }

        // If we made it this far, then no index was found. Return null (effectively erroring)
        return null;
    }
}
