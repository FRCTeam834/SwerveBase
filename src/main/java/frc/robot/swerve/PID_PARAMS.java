/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.swerve;

public class PID_PARAMS {
    public double P;
    public double I;
    public double D;
    public double SFF;
    public double VFF;
    public int I_ZONE;
    public double PEAK_OUTPUT;
    
    // Creating a new set of PID Parameters
    public PID_PARAMS(double P, double I, double D, double SFF, double VFF, int I_ZONE, double PEAK_OUTPUT){
        this.P = P;
        this.I = I;
        this.D = D;
        this.SFF = SFF;
        this.VFF = VFF;
        this.I_ZONE = I_ZONE;
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }

    // Change the parameters with the new values
    public void setPIDParams(PID_PARAMS params) {
        this.P = params.P;
        this.I = params.I;
        this.D = params.D;
        this.SFF = params.SFF;
        this.VFF = params.VFF;
        this.I_ZONE = params.I_ZONE;
        this.PEAK_OUTPUT = params.PEAK_OUTPUT;
    }

    // Set only the peak output (needed when the driver profile changes)
    public void setPeakOutput(double PEAK_OUTPUT) {
        this.PEAK_OUTPUT = PEAK_OUTPUT;
    }
    
}