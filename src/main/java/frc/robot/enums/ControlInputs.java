/**
 * @author Christian Piper (CAP1Sup)
 * @since 11/2/21
 */
package frc.robot.enums;

public enum ControlInputs {

  // Main definition of types (each should have their own index)
  JOYSTICKS(0),
  XBOX(1);

  // The index of each output type
  private int index;

  /**
   * Creates a new output type
   *
   * @param index The index to set each output type to
   */
  private ControlInputs(int index) {
    this.index = index;
  }

  /**
   * Gets the integer value for the specified output type
   *
   * @return int index of output type
   */
  public int getInt() {
    return this.index;
  }

  /**
   * Looks up an index and returns the corresponding output type
   *
   * @param index The index that you wish to look up
   * @return Output type matching index
   */
  public static ControlInputs fromInt(int index) {

    // Loop through all of the possible output types
    for (ControlInputs type : values()) {

      // Return the type if the index matches
      if (type.getInt() == index) {
        return type;
      }
    }

    // If we made it this far, then no index was found. Return null (effectively erroring)
    return null;
  }
}
