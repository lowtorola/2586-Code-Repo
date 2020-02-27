package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax winch = new CANSparkMax(
        ClimbConstants.kWinchID, MotorType.kBrushless);
    public ClimbSubsystem() {
        winch.setIdleMode(IdleMode.kBrake);
    }

    public void setWinch() {
        winch.set(0.2);
    }

    public void stopWinch() {
        winch.set(0);
    }
/*    public void setWinch(double speed) {
  *      winch.set(speed);
  *  }
    */
}