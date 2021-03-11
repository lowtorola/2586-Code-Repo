
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

  //  private Spark feederMotor = new Spark(ShooterConstants.kFeederMotorID);

    private final CANEncoder shooterEncoder = shooterMotor.getEncoder();

    DigitalInput feederBbRec = new DigitalInput(0);
    DigitalInput shooterExitBbRec = new DigitalInput(2);

    private SimpleMotorFeedforward feedforward = 
    new SimpleMotorFeedforward(ShooterConstants.ksVolts,
                               ShooterConstants.kvVoltSecondsPerRotation,
                               ShooterConstants.kaVoltSecondsSquaredPerRotation);

    private DoubleSolenoid hoodPiston = new DoubleSolenoid(
      ShooterConstants.kHoodPistonPorts[0], ShooterConstants.kHoodPistonPorts[1]);

    public ShooterSubsystem() {
    }

      // -- methods for setting shooter in motion

      public void runShooter(double output) {
       shooterMotor.set(output);
      // System.out.println(output);
      }

      public void setShooterVolts(double outputVolts) {
        shooterMotor.setVoltage(outputVolts);
      }

      public void stopShooter() {
        shooterMotor.set(0);
      }

      public void runFeeder() {
       // feederMotor.set(ShooterConstants.kFeederMotorSpeed);
      }

      public void preloadFeeder() {
       // feederMotor.set(ShooterConstants.kFeederPreloadSpeed);
      }

      public void stopFeeder() {
      //  feederMotor.set(0);
      }

      public void extendHood() {
        hoodPiston.set(DoubleSolenoid.Value.kForward);
      }

      public void retractHood() {
        hoodPiston.set(DoubleSolenoid.Value.kReverse);
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

    /**
     * Calculates a dynamic shooter feed forward
     * @param targetRPM the current target RPM
     * @return returns a dynamic feed forward for our shooter
     */
    public double getFeedForward(double targetRPM) {
        return feedforward.calculate(targetRPM);
    }

    /**
     * 
     * @param targetDist the current distance from the target, "x" in the method's quadratic ax^2+bx+c
     * @return the RPM the shooter needs to spin at to reach the target
     */
    public double getTargetRPM(double targetDist) {
        if(targetDist < ShooterConstants.kDistanceSwitcher) {
          return
          (ShooterConstants.kQuadraticNear * targetDist) + (ShooterConstants.kLinearNear * targetDist) + ShooterConstants.kConstantNear;
        } else {
          return 
          (ShooterConstants.kQuadraticFar * targetDist) + (ShooterConstants.kLinearFar * targetDist) + ShooterConstants.kConstantFar;
        }
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
