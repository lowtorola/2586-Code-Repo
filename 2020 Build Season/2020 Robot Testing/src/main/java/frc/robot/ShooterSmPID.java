package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class ShooterSmPID extends CANPIDController {

    static double KP = 0;
    static double KI = 0;
    static double KD = 0;
    static double KF = 0;
    static double MAXVEL = 2500;
    
    public ShooterSmPID(CANSparkMax device) {
        super(device);
    }

    

}