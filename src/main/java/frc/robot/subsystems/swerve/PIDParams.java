/**
 * @author Christian Piper (@CAP1Sup)
 * @since 5/22/20
 */

package frc.robot.subsystems.swerve;

/**
 *  Class that organizes gains used when assigning values to slots
 */
public class PIDParams {

    // The values of the PID loop
    public double P;
    public double I;
    public double D;
    public double FF;
    public double I_ZONE;
    public double PEAK_OUTPUT;

    // Creating a new set of PID Parameters
    public PIDParams(double P, double I, double D, double FF, double I_ZONE, double PEAK_OUTPUT){
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
        this.I_ZONE = I_ZONE;
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }

    // Change the parameters with the new values
    public void setPIDParams(PIDParams params) {
        this.P = params.P;
        this.I = params.I;
        this.D = params.D;
        this.FF = params.FF;
        this.I_ZONE = params.I_ZONE;
        this.PEAK_OUTPUT = params.PEAK_OUTPUT;
    }

    // Set only the peak output (needed when the driver profile changes)
    public void setPeakOutput(double PEAK_OUTPUT) {
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }

}
