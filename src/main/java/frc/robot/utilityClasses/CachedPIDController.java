/** @author Christian Piper (@CAP1Sup) */
package frc.robot.utilityClasses;

// Imports
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class CachedPIDController extends CANPIDController {

    // Variables for caching
    double previousValue = 0;
    ControlType previousControlType = ControlType.kDutyCycle;
    CANError previousCanError = CANError.kOk;

    /**
     * Creates a new Cached Spark Max object. This is useful for preventing the sending of repeated
     * values to the Spark Maxes. It will help to reduce the load on the CAN bus (only a set amount
     * of information can be sent across the CAN bus in a given time).
     *
     * @param deviceID The CAN ID of the motor controller
     * @param type The type of motor being used
     */
    @SuppressWarnings("all")
    public CachedPIDController(CANSparkMax device) {
        // ! IGNORE THIS WARNING, there's no other way to create the new
        // controller. Even REV themselves use it
        super(device);
    }

    /**
     * Sets the reference with caching (repeated values are not set)
     *
     * @param value The value to set depending on the control mode. For basic duty cycle control
     *     this should be a value between -1 and 1 Otherwise: Voltage Control: Voltage (volts)
     *     Velocity Control: Velocity (RPM) Position Control: Position (Rotations) Current Control:
     *     Current (Amps). Native units can be changed using the setPositionConversionFactor() or
     *     setVelocityConversionFactor() methods of the CANEncoder class
     * @param ctrl is the control type
     * @return Set to REV_OK if successful
     */
    public CANError setReference(double value, ControlType ctrl) {

        // Check if the values have been changed
        // We use separated if loops to reduce the typical amount of checks needed
        // Values change the most, so we check those first, then control types
        if (value != previousValue) {
            if (!ctrl.equals(previousControlType)) {

                // We need to send the values
                // First save the current values for the next cycle
                previousValue = value;
                previousControlType = ctrl;

                // Send the set command, saving the output
                previousCanError = super.setReference(value, ctrl);
            }
        }

        // Return the current CAN error
        // We can just return the previous error each time as the variable is updated
        // when unique values are sent
        return previousCanError;
    }
}
