/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.swerve;

public class PID_PARAMETERS {
    public double P;
    public double I;
    public double D;
    public double SFF;
    public double VFF;
    public int I_ZONE;
    public double PEAK_OUTPUT;
    
    // Creating a new set of PID Parameters
    public PID_PARAMETERS(double P, double I, double D, double SFF, double VFF, int I_ZONE, double PEAK_OUTPUT){
        this.P = P;
        this.I = I;
        this.D = D;
        this.SFF = SFF;
        this.VFF = VFF;
        this.I_ZONE = I_ZONE;
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }

    // Change the parameters with the new values
    public void setPIDParams(PID_PARAMETERS parameters) {
        this.P = parameters.P;
        this.I = parameters.I;
        this.D = parameters.D;
        this.SFF = parameters.SFF;
        this.VFF = parameters.VFF;
        this.I_ZONE = parameters.I_ZONE;
        this.PEAK_OUTPUT = parameters.PEAK_OUTPUT;
    }

    // Set only the peak output (needed when the driver profile changes)
    public void setPeakOutput(double PEAK_OUTPUT) {
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }
    
}