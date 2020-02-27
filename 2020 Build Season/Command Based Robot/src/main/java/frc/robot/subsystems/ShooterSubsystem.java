package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends PIDSubsystem {

    private final CANSparkMax shooterMotor = new CANSparkMax(
        ShooterConstants.kShooterMotorID, ShooterConstants.kShooterMotorType);

    private final Spark feederMotor = new Spark(ShooterConstants.kFeederMotorID);

    private final CANEncoder shooterEncoder = shooterMotor.getEncoder();

    private final DigitalInput feederBbRec = new DigitalInput(ShooterConstants.kFeederBbRecPort);
    private final DigitalOutput feederBbBlast = new DigitalOutput(ShooterConstants.kFeederBbBlastPort);
    private final DigitalInput shooterExitBbRec = new DigitalInput(ShooterConstants.kShooterExitRecPort);
    private final DigitalOutput shooterExitBbBlast = new DigitalOutput(ShooterConstants.kShooterExitBlastPort);

    public ShooterSubsystem() {
        super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
        getController().setTolerance(ShooterConstants.kToleranceRPM);
        setSetpoint(ShooterConstants.kTargetRPM);
      }

      @Override
    public void useOutput(double setpoint, double output) {
        shooterMotor.set(output / ShooterConstants.kMaxRPM);
    }
    
    public void printBB() {
      SmartDashboard.putBoolean("Feeder BB", feederBbRec.get());
      SmartDashboard.putBoolean("Exit BB", getShooterExitBB());
       
    }

    public boolean getFeederBB() {
      return feederBbRec.get();
    }

    public boolean getShooterExitBB() {
      return shooterExitBbRec.get();
    }

      @Override
    public double getMeasurement() {
        return (shooterEncoder.getVelocity());
      }

      public boolean isAtSpeed() {
        return (Math.abs(ShooterConstants.kTargetRPM - shooterEncoder.getVelocity()) <  ShooterConstants.kToleranceRPM); // TODO: change to full speed values
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

      public void printShooterRPM() {
        SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getVelocity());
      }

}