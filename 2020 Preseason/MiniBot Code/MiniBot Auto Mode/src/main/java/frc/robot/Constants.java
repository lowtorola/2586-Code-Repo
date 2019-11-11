package frc.robot;

public class Constants {

    // Auto PID turn
    static double kP = 5e-5; 
    static double kI = 1e-6;
    static double kD = 0; 
    static double kIz = 0; 
    static double kFF = 0.000156; 
    static double kMaxOutput = 1; 
    static double kMinOutput = -1;
    static double maxRPM = 5700;
    static int smartMotionSlot = 0;
    static int maxVel = 2000; // rpm
    static int minVel = -2000; // rpm
    static int maxAcc = 1500;
    static int allowedErr = 3;

}