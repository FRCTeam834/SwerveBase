/**
 * @author Christian Piper (@CAP1Sup)
 * @since 10/23/21
 */
package frc.robot.DriverProfiles;

// Imports
import frc.robot.enums.JoystickOutputTypes;

/**
 * @param deadzone The deadzone of the joysticks. Joystick values range from -1 to 1, so a 0.1
 *     deadzone is usually sufficient to deal with any wobble
 * @param outputTypes The type of output curve for the joysticks. Can be any value of {@link
 *     frc.robot.enums.JoystickOutputTypes}
 * @param rampConst The ramp rate of the quatratic function (only used if JOYSTICK_OUTPUT_TYPE ==
 *     ZEROED_QUAD_LINEAR). The a value in this graph: https://www.desmos.com/calculator/5lqgnstb1k.
 *     Note that this value MUST BE CHECKED, otherwise crazy things will happen (not so good!)
 */
public class JoystickParams {

  // Private variables
  private double deadzone, rampConst;
  private JoystickOutputTypes outputType;
  private double crossoverVal, linearSlope;

  /**
   * Creates a new set of joystick parameters
   *
   * @param deadzone The deadzone of the joysticks. Joystick values range from -1 to 1, so a 0.1
   *     deadzone is usually sufficient to deal with any wobble
   * @param outputType The type of output curve for the joysticks. Can be any value of {@link
   *     frc.robot.enums.JoystickOutputTypes}
   */
  public JoystickParams(double deadzone, JoystickOutputTypes outputType) {
    setJoystickParams(deadzone, outputType);
  }

  /**
   * Creates a new set of joystick parameters
   *
   * @param deadzone The deadzone of the joysticks. Joystick values range from -1 to 1, so a 0.1
   *     deadzone is usually sufficient to deal with any wobble
   * @param outputType The type of output curve for the joysticks. Can be any value of {@link
   *     frc.robot.enums.JoystickOutputTypes}
   * @param rampConst The ramp rate of the quadratic function (only used if outputType ==
   *     ZEROED_QUAD_LINEAR). The a value in this graph:
   *     https://www.desmos.com/calculator/5lqgnstb1k. Note that this value MUST BE CHECKED,
   *     otherwise crazy things will happen (not so good!)
   */
  public JoystickParams(double deadzone, JoystickOutputTypes outputType, double rampConst) {
    setJoystickParams(deadzone, outputType, rampConst);
  }

  /**
   * Sets the parameters for the joystick
   *
   * @param deadzone The deadzone of the joysticks. Joystick values range from -1 to 1, so a 0.1
   *     deadzone is usually sufficient to deal with any wobble
   * @param outputType The type of output curve for the joysticks. Can be any value of {@link
   *     frc.robot.enums.JoystickOutputTypes}
   */
  public void setJoystickParams(double deadzone, JoystickOutputTypes outputType) {

    // Make sure that the output type isn't ZEROED_QUAD_LINEAR
    if (outputType == JoystickOutputTypes.ZEROED_QUAD_LINEAR) {

      // We can't use ZEROED_QUAD_LINEAR without a proper ramp rate term, so fall back to
      // ZEROED_LINEAR
      outputType = JoystickOutputTypes.ZEROED_LINEAR;
      System.out.println(
          "WARNING: No ramp rate specified with ZEROED_QUAD_LINEAR, falling back to ZEROED_LINEAR!");
    }

    // Save the values of the parameters
    this.deadzone = deadzone;
    this.outputType = outputType;
  }

  /**
   * Sets the parameters for the joystick
   *
   * @param deadzone The deadzone of the joysticks. Joystick values range from -1 to 1, so a 0.1
   *     deadzone is usually sufficient to deal with any wobble
   * @param outputType The type of output curve for the joysticks. Can be any value of {@link
   *     frc.robot.enums.JoystickOutputTypes}
   * @param rampConst The ramp rate of the quadratic function (only used if outputType ==
   *     ZEROED_QUAD_LINEAR). The a value in this graph:
   *     https://www.desmos.com/calculator/5lqgnstb1k. Note that this value MUST BE CHECKED,
   *     otherwise crazy things will happen (not so good!)
   */
  public void setJoystickParams(double deadzone, JoystickOutputTypes outputType, double rampConst) {

    // Set the basic parameters
    setJoystickParams(deadzone, outputType);

    // We only need to do the fancy math if we're using the fancy scaling method
    if (outputType == JoystickOutputTypes.ZEROED_QUAD_LINEAR) {

      // Create short little variables to simplify the formula
      // Wastes time, but well worth it in readability,
      // especially because this only has to be computed once or twice
      double a = rampConst;
      double t = deadzone;

      // Compute the joystick crossover threshold (expensive to compute, therefore should only be
      // done once and saved)
      // This value is based on a bunch of calculus and algebra
      // "Don't worry about it, it just works"
      // Based on this: https://www.desmos.com/calculator/5lqgnstb1k
      this.crossoverVal = -(Math.sqrt(a * (a * t * t - 2 * a * t + a - 1)) - a) / a;
      this.linearSlope = (a * Math.pow(crossoverVal - t, 2) - 1) / (crossoverVal - 1);
    }
  }

  /**
   * Returns the deadzone of the joysticks (a threshold of 0.1 should be inactive between -0.1 and
   * 0.1)
   *
   * @return Joystick threshold
   */
  public double getDeadzone() {
    return this.deadzone;
  }

  /**
   * Returns the ramp rate of the joysticks (only if using ZEROED_QUAD_LINEAR as the output type)
   *
   * @return "A" term of quadratic equation
   */
  public double getRampRate() {
    if (this.outputType == JoystickOutputTypes.ZEROED_QUAD_LINEAR) {
      return this.rampConst;
    } else {
      return 0;
    }
  }

  /**
   * Returns the precomputed crossover value of the joysticks (only if using ZEROED_QUAD_LINEAR as
   * the output type) Simpifies processing and increases execution speed
   *
   * @return Precomputed crossover value
   */
  public double getCrossoverValue() {
    if (this.outputType == JoystickOutputTypes.ZEROED_QUAD_LINEAR) {
      return this.crossoverVal;
    } else {
      return 0;
    }
  }

  /**
   * Returns the precomputed linear slope of the joysticks (only if using ZEROED_QUAD_LINEAR as the
   * output type)
   *
   * @return Precomputed linear slope
   */
  public double getLinearSlope() {
    if (this.outputType == JoystickOutputTypes.ZEROED_QUAD_LINEAR) {
      return this.linearSlope;
    } else {
      return 0;
    }
  }

  /**
   * Returns output mode of the joysticks (a value of {@link frc.robot.enums.JoystickOutputTypes})
   *
   * @return Joystick threshold
   */
  public JoystickOutputTypes getOutputType() {
    return this.outputType;
  }
}
