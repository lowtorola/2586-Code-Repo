
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor = new CANSparkMax(
      ShooterConstants.kShooterMotorID, ShooterConstants.kShooterMotorType);

    private final CANSparkMax beltMotor = new CANSparkMax(
      ShooterConstants.kBeltMotorID, ShooterConstants.kShooterMotorType);

    private final Spark feederMotor = new Spark(ShooterConstants.kFeederMotorID);

    private final DoubleSolenoid hoodPiston = new DoubleSolenoid(
      ShooterConstants.kHoodPistonID[0], ShooterConstants.kHoodPistonID[1]);


    private final CANEncoder shooterEncoder = shooterMotor.getEncoder();

    private final DigitalInput feederBbRec = new DigitalInput(ShooterConstants.kFeederBbRecPort);
    private final DigitalInput shooterExitBbRec = new DigitalInput(ShooterConstants.kShooterExitRecPort);

    private SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(ShooterConstants.ksVolts,
                               ShooterConstants.kvVoltSecondsPerRotation,
                               ShooterConstants.kaVoltSecondsSquaredPerRotation);

    public ShooterSubsystem() {
      hoodPiston.set(DoubleSolenoid.Value.kReverse);
    }

      // -- methods for setting shooter in motion

      public void runShooter(double output) {
       shooterMotor.set(output);
      // System.out.println(output);
      }

      public void stopShooter() {
        shooterMotor.set(0);
        System.out.println("shooter stopped");
      }

      public void runFeed() {
        feederMotor.set(ShooterConstants.kFeederMotorSpeed);
        beltMotor.set(ShooterConstants.kBeltMotorSpeed); // TODO: test to see optimal belt speed
      }

      public void preloadShooter() {
        feederMotor.set(ShooterConstants.kFeederPreloadSpeed);
        beltMotor.set(ShooterConstants.kBeltPreloadSpeed);
      }

      public void stopFeed() {
        feederMotor.set(0);
        beltMotor.set(0);
      }

      public void extendHood() {
        hoodPiston.set(DoubleSolenoid.Value.kForward);
        System.out.println("extend hood");
      }

      public void retractHood() {
        hoodPiston.set(DoubleSolenoid.Value.kReverse);
        System.out.println("retract hood");
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
