
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    
    private final CANSparkMax shooterMotor = new CANSparkMax(
        ShooterConstants.kShooterMotorID, ShooterConstants.kShooterMotorType);

    private final Spark feederMotor = new Spark(ShooterConstants.kFeederMotorID);

    private final CANEncoder shooterEncoder = shooterMotor.getEncoder();

    private final DigitalInput feederBbRec = new DigitalInput(ShooterConstants.kFeederBbRecPort);
    private final DigitalInput shooterExitBbRec = new DigitalInput(ShooterConstants.kShooterExitRecPort);

    private SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(ShooterConstants.ksVolts,
                               ShooterConstants.kvVoltSecondsPerRotation,
                               ShooterConstants.kaVoltSecondsSquaredPerRotation);

    public ShooterSubsystem() {
    }

      // -- methods for setting shooter in motion

      public void runShooter(double output) {
       shooterMotor.set(output);
      // System.out.println(output);
      }

      public void stopShooter() {
        shooterMotor.set(0);
      }

      public void runFeeder() {
        feederMotor.set(ShooterConstants.kFeederMotorSpeed);
      }

      public void preloadFeeder() {
        feederMotor.set(ShooterConstants.kFeederPreloadSpeed);
      }

      public void stopFeeder() {
        feederMotor.set(0);
      }

      // -- methods for getting values: static and dynamic

      public boolean getFeederBB() {
        return feederBbRec.get();
      }
  
    public boolean getShooterExitBB() {
        return shooterExitBbRec.get();
      }

    public double getMeasurement() {
        return shooterMotor.getBusVoltage();
    }

    public double getFeedForward() {
        return feedforward.calculate(ShooterConstants.kTargetRPM) / ShooterConstants.kMaxRPM;
    }

    public double getTargetRPM() {
        return ShooterConstants.kTargetRPM; // When adjustable shooter is done, this will need to be dynamic
    }

    public double getToleranceRPM() {
        return ShooterConstants.kToleranceRPM;
    }

    public boolean isAtSpeed() {
        return (Math.abs(ShooterConstants.kTargetRPM - shooterEncoder.getVelocity()) <  ShooterConstants.kToleranceRPM); // TODO: change to full speed values
    }

      // -- print outs

      public void printShooterRPM() {
        SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Output", shooterMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter Voltage", shooterMotor.get());
      }

      public void printBB() {
        SmartDashboard.putBoolean("Feeder BB", feederBbRec.get());
        SmartDashboard.putBoolean("Exit BB", getShooterExitBB());
     }

}
