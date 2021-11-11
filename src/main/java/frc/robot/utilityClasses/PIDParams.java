/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/22/20
 */

package frc.robot.utilityClasses;

import com.revrobotics.ControlType;

/**
 *  Class that organizes gains used when assigning values to slots
 */
public class PIDParams {

    // The values of the PID loop
    public double kP;
    public double kI;
    public double kD;
    public double kFF;
    public double iZone;
    public double maxOutput;
    public ControlType controlType;


    // Creates a new set of PID Parameters
    public PIDParams(double kP, double kI, double kD, double kFF, double iZone, double maxOutput, ControlType ctrlType){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.iZone = iZone;
        this.maxOutput = maxOutput;
        this.controlType = ctrlType;
    }


    // Creates a new set of PID Parameters
    public PIDParams(double kP, double kI, double kD, double kFF, double maxOutput, ControlType ctrlType){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
        this.iZone = 0; // (Disabled when set to 0)
        this.maxOutput = maxOutput;
        this.controlType = ctrlType;
    }


    // Change the parameters with the new values
    public void setPIDParams(PIDParams params) {
        this.kP = params.kP;
        this.kI = params.kI;
        this.kD = params.kD;
        this.kFF = params.kFF;
        this.iZone = params.iZone;
        this.maxOutput = params.maxOutput;
        this.controlType = params.controlType;
    }


    // Set only the max output (needed when the driver profile changes)
    public void setPeakOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }
}
