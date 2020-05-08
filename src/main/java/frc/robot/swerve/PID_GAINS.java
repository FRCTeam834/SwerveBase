/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot.swerve;

public class PID_GAINS {
    public final double P;
    public final double I;
    public final double D;
    public final double F;
    public final int I_ZONE;
    public final double PEAK_OUTPUT;
    
    public PID_GAINS(double P, double I, double D, double F, int I_ZONE, double PEAK_OUPUT){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
        this.I_ZONE = I_ZONE;
        this.PEAK_OUTPUT = PEAK_OUPUT;
    }
}