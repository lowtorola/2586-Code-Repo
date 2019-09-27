package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

        private static final String kDefaultAuto = "Default";
        private static final String kCustomAuto = "My Auto";
        private String m_autoSelected;
        private final SendableChooser<String> m_chooser = new SendableChooser<>();

        private WPI_TalonSRX m_Left0 = new WPI_TalonSRX(1);
        private WPI_TalonSRX m_Left1 = new WPI_TalonSRX(2);
        private WPI_TalonSRX m_Right0 = new WPI_TalonSRX(4);
        private WPI_TalonSRX m_Right1 = new WPI_TalonSRX(3);
        private SpeedControllerGroup m_LeftMotors = new SpeedControllerGroup(m_Left0, m_Left1);
        private SpeedControllerGroup m_RightMotors = new SpeedControllerGroup(m_Right0, m_Right1);
        private DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

        private Joystick m_Controller = new Joystick(0);

        private boolean m_LimelightHasValidTarget = false;
        // private double m_LimelightDriveCommand = 0.0;
        private double m_LimelightSteerCommand = 0.0;

        /**
         * This function is run when the robot is first started up and should be used
         * for any initialization code.
         */
        @Override
        public void robotInit() {
                m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
                m_chooser.addOption("My Auto", kCustomAuto);
                SmartDashboard.putData("Auto choices", m_chooser);

                m_Left0.setInverted(true);
                m_Left1.setInverted(true);

        }

        /**
         * This function is called every robot packet, no matter the mode. Use this for
         * items like diagnostics that you want ran during disabled, autonomous,
         * teleoperated and test.
         *
         * <p>
         * This runs after the mode specific periodic functions, but before LiveWindow
         * and SmartDashboard integrated updating.
         */
        @Override
        public void robotPeriodic() {
        }

        /**
         * This autonomous (along with the chooser code above) shows how to select
         * between different autonomous modes using the dashboard. The sendable chooser
         * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
         * remove all of the chooser code and uncomment the getString line to get the
         * auto name from the text box below the Gyro
         *
         * <p>
         * You can add additional auto modes by adding additional comparisons to the
         * switch structure below with additional strings. If using the SendableChooser
         * make sure to add them to the chooser code above as well.
         */
        @Override
        public void autonomousInit() {
                m_autoSelected = m_chooser.getSelected();
        }

        /**
         * This function is called periodically during autonomous.
         */
        @Override
        public void autonomousPeriodic() {
        }

        /**
         * This function is called periodically during operator control.
         */
        @Override
        public void teleopPeriodic() {

                double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
                double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

                // System.out.println(tx);

                double drive_power = -m_Controller.getRawAxis(2);
                double drive_rotate = m_Controller.getRawAxis(1);
                boolean autoAim = m_Controller.getRawButton(1);

                double leftCommand = 0;
                double rightCommand = 0;

                float Kp = -0.05f;
                float min_command = 0.05f;

               

                if (m_Controller.getRawButton(1)) {
                        float heading_error = (float) -tx;
                        float steering_adjust = 0.0f;
                        if (tx > 1.0) {
                                steering_adjust = Kp * heading_error - min_command;
                        } else if (tx < 1.0) {
                                steering_adjust = Kp * heading_error + min_command;
                        }

                        leftCommand += steering_adjust;
                        rightCommand -= steering_adjust;

                        System.out.println(steering_adjust);
                       
                        m_Drive.tankDrive(-leftCommand, rightCommand);
                }
                else{
                        m_Drive.arcadeDrive(drive_power, drive_rotate);
                }

        }

}