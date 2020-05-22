/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.swerve;

public class PID_PARAMETERS {
    public double P;
    public double I;
    public double D;
    public double FF;
    public int I_ZONE;
    public double PEAK_OUTPUT;
    
    public PID_PARAMETERS(double P, double I, double D, double FF, int I_ZONE, double PEAK_OUPUT){
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
        this.I_ZONE = I_ZONE;
        this.PEAK_OUTPUT = PEAK_OUPUT;
    }

    public void setPIDParams(PID_PARAMETERS parameters) {
        this.P = parameters.P;
        this.I = parameters.I;
        this.D = parameters.D;
        this.FF = parameters.FF;
        this.I_ZONE = parameters.I_ZONE;
        this.PEAK_OUTPUT = parameters.PEAK_OUTPUT;
    }
    
}