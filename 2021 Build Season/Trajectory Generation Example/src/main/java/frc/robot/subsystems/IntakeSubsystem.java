package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    Spark intakeRoller = new Spark(IntakeConstants.kIntakeID);

    DoubleSolenoid intakeDeploy = new DoubleSolenoid(
        IntakeConstants.kIntakePistonID[0], IntakeConstants.kIntakePistonID[1]);

 public IntakeSubsystem() {
    intakeDeploy.set(DoubleSolenoid.Value.kForward); // TODO: change to reverse
 }

 public void startIntake() {
     intakeRoller.set(IntakeConstants.kIntakeSpeed);
 }

 public void stopIntake() {
     intakeRoller.set(0);
 }

 public void deployIntake() {
     intakeDeploy.set(DoubleSolenoid.Value.kForward);
 }

 public void retractIntake() {
     intakeDeploy.set(DoubleSolenoid.Value.kReverse);
 }
}