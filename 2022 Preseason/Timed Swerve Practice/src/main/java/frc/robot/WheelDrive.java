package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

public class WheelDrive {
    
    private WPI_TalonFX angleMotor;
    private WPI_TalonFX speedMotor;
    private CANCoder turnEncoder;
    private PIDController pidController;
    private final double MAX_VOLTS = 4.95;

    public WheelDrive(int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new WPI_TalonFX(angleMotor);
        this.speedMotor = new WPI_TalonFX(speedMotor);
        this.turnEncoder = new CANCoder(encoder);
        pidController = new PIDController(0.3, 0.003, 0);
        // pidController = new PIDController(1, 0, 0, new AnalogInput(encoder), angleMotor);
        pidController.enableContinuousInput(-180, 180);
    }

    public void drive (double speed, double angle) {
        speedMotor.set(TalonFXControlMode.PercentOutput, speed);

        
        angleMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(pidController.calculate(turnEncoder.getAbsolutePosition() * (Math.PI/180), angle), -0.2, 0.2)); // need to set output to -180,180 in phoenix tuner
    }

    public double getEncValue() {
        return turnEncoder.getAbsolutePosition();
    }
    public double getTurnMotorOutput() {
        return angleMotor.getMotorOutputPercent();
    }
    public double getDriveMotorOutput() {
        return speedMotor.getMotorOutputPercent();
    }

}
