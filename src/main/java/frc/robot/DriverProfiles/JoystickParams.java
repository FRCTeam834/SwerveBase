package frc.robot.DriverProfiles;

import frc.robot.enums.JOYSTICK_OUTPUT_TYPES;

/**
 * @param JOYSTICK_DEADZONE     The deadzone of the joysticks. Joystick values
 *                              range from -1 to 1, so a 0.1 deadzone is usually
 *                              sufficient to deal with any wobble
 * @param JOYSTICK_OUTPUT_TYPES The type of output curve for the joysticks. Can be any value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES}
 * @param JOYSICK_RAMP_CONST    The ramp rate of the quatratic function (only
 *                              used if JOYSTICK_OUTPUT_TYPE ==
 *                              ZEROED_QUAD_LINEAR). The a value in this graph:
 *                              https://www.desmos.com/calculator/5lqgnstb1k.
 *                              Note that this value MUST BE CHECKED, otherwise
 *                              crazy things will happen (not so good!)
 */
public class JoystickParams {

    // Private variables
    private double JOYSTICK_DEADZONE, JOYSTICK_RAMP_CONST;
    private JOYSTICK_OUTPUT_TYPES JOYSTICK_OUTPUT_TYPE;
    private double crossoverVal, linearSlope;


    /**
     * Creates a new set of joystick parameters
     * @param JOYSTICK_DEADZONE     The deadzone of the joysticks. Joystick values
     *                              range from -1 to 1, so a 0.1 deadzone is usually
     *                              sufficient to deal with any wobble
     * @param JOYSTICK_OUTPUT_TYPES The type of output curve for the joysticks. Can be any value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES}
     * @param JOYSICK_RAMP_CONST    The ramp rate of the quatratic function (only
     *                              used if JOYSTICK_OUTPUT_TYPE ==
     *                              ZEROED_QUAD_LINEAR). The a value in this graph:
     *                              https://www.desmos.com/calculator/5lqgnstb1k.
     *                              Note that this value MUST BE CHECKED, otherwise
     *                              crazy things will happen (not so good!)
     */
    public JoystickParams(double JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPES JOYSTICK_OUTPUT_TYPE) {
        setJoystickParams(JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPE);
    }


    /**
     * Creates a new set of joystick parameters
     * @param JOYSTICK_DEADZONE     The deadzone of the joysticks. Joystick values
     *                              range from -1 to 1, so a 0.1 deadzone is usually
     *                              sufficient to deal with any wobble
     * @param JOYSTICK_OUTPUT_TYPES The type of output curve for the joysticks. Can be any value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES}
     * @param JOYSICK_RAMP_CONST    The ramp rate of the quatratic function (only
     *                              used if JOYSTICK_OUTPUT_TYPE ==
     *                              ZEROED_QUAD_LINEAR). The a value in this graph:
     *                              https://www.desmos.com/calculator/5lqgnstb1k.
     *                              Note that this value MUST BE CHECKED, otherwise
     *                              crazy things will happen (not so good!)
     */
    public JoystickParams(double JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPES JOYSTICK_OUTPUT_TYPE, double JOYSTICK_RAMP_CONST) {
        setJoystickParams(JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPE, JOYSTICK_RAMP_CONST);
    }


    /**
     * Sets the parameters for the joystick
     * @param JOYSTICK_DEADZONE     The deadzone of the joysticks. Joystick values
     *                              range from -1 to 1, so a 0.1 deadzone is usually
     *                              sufficient to deal with any wobble
     * @param JOYSTICK_OUTPUT_TYPES The type of output curve for the joysticks. Can be any value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES}
     */
    public void setJoystickParams(double JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPES JOYSTICK_OUTPUT_TYPE) {

        // Make sure that the output type isn't ZEROED_QUAD_LINEAR
        if (JOYSTICK_OUTPUT_TYPE == JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR) {

            // We can't use ZEROED_QUAD_LINEAR without a proper ramp rate term, so fall back to ZEROED_LINEAR
            JOYSTICK_OUTPUT_TYPE = JOYSTICK_OUTPUT_TYPES.ZEROED_LINEAR;
            System.out.println("WARNING: No ramp rate specified with ZEROED_QUAD_LINEAR, falling back to ZEROED_LINEAR!");
        }

        // Save the values of the parameters
        this.JOYSTICK_DEADZONE     = JOYSTICK_DEADZONE;
        this.JOYSTICK_OUTPUT_TYPE  = JOYSTICK_OUTPUT_TYPE;
    }


    /**
     * Sets the parameters for the joystick
     * @param JOYSTICK_DEADZONE     The deadzone of the joysticks. Joystick values
     *                              range from -1 to 1, so a 0.1 deadzone is usually
     *                              sufficient to deal with any wobble
     * @param JOYSTICK_OUTPUT_TYPES The type of output curve for the joysticks. Can be any value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES}
     * @param JOYSICK_RAMP_CONST    The ramp rate of the quatratic function (only
     *                              used if JOYSTICK_OUTPUT_TYPE ==
     *                              ZEROED_QUAD_LINEAR). The a value in this graph:
     *                              https://www.desmos.com/calculator/5lqgnstb1k.
     *                              Note that this value MUST BE CHECKED, otherwise
     *                              crazy things will happen (not so good!)
     */
    public void setJoystickParams(double JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPES JOYSTICK_OUTPUT_TYPE, double JOYSTICK_RAMP_CONST) {

        // Set the basic parameters
        setJoystickParams(JOYSTICK_DEADZONE, JOYSTICK_OUTPUT_TYPE);

        // We only need to do the fancy math if we're using the fancy scaling method
        if (JOYSTICK_OUTPUT_TYPE == JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR) {

            // Create short little variables to simplify the formula
            // Wastes time, but well worth it in readability,
            // especially because this only has to be computed once or twice
            double a = JOYSTICK_RAMP_CONST;
            double t = JOYSTICK_DEADZONE;

            // Compute the joystick crossover threshold (expensive to compute, therefore should only be done once and saved)
            // This value is based on a bunch of calculus and algebra
            // "Don't worry about it, it just works"
            // Based on this: https://www.desmos.com/calculator/5lqgnstb1k
            double b = - (Math.sqrt(a * (a*t*t - 2*a*t + a - 1)) - a) / a;
            this.linearSlope = (a*(b-t)*(b-t) - 1) / (b - 1);

            // Save the crossover value
            this.crossoverVal = b;
        }
    }


    /**
     * Returns the deadzone of the joysticks (a threshold of 0.1 should be inactive between -0.1 and 0.1)
     * @return Joystick threshold
     */
    public double getDeadzone() {
        return this.JOYSTICK_DEADZONE;
    }


    /**
     * Returns the ramp rate of the joysticks (only if using ZEROED_QUAD_LINEAR as the output type)
     * @return "A" term of quadratic equation
     */
    public double getRampRate() {
        if (this.JOYSTICK_OUTPUT_TYPE == JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR) {
            return this.JOYSTICK_RAMP_CONST;
        }
        else {
            return 0;
        }
    }


    /**
     * Returns the precomputed crossover value of the joysticks (only if using ZEROED_QUAD_LINEAR as the output type)
     * Simpifies processing and increases execution speed
     * @return Precomputed crossover value
     */
    public double getCrossoverValue() {
        if (this.JOYSTICK_OUTPUT_TYPE == JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR) {
            return this.crossoverVal;
        }
        else {
            return 0;
        }
    }


    /**
     * Returns the precomputed linear slope of the joysticks (only if using ZEROED_QUAD_LINEAR as the output type)
     * @return Precomputed linear slope
     */
    public double getLinearSlope() {
        if (this.JOYSTICK_OUTPUT_TYPE == JOYSTICK_OUTPUT_TYPES.ZEROED_QUAD_LINEAR) {
            return this.linearSlope;
        }
        else {
            return 0;
        }
    }


    /**
     * Returns output mode of the joysticks (a value of {@link frc.robot.enums.JOYSTICK_OUTPUT_TYPES})
     * @return Joystick threshold
     */
    public JOYSTICK_OUTPUT_TYPES getOutputType() {
        return this.JOYSTICK_OUTPUT_TYPE;
    }
}
