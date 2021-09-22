package frc.robot.subsystems;

import com.mach.LightDrive.LightDriveCAN;

import java.awt.Color; //Predefined colors and routines

public class LEDSubsystem {
    
    private LightDriveCAN ld_can = new LightDriveCAN();

    public LEDSubsystem() {
        ld_can.SetColor(1, Color.ORANGE, 0.9);
        ld_can.SetColor(4, Color.MAGENTA, 0.9);
        ld_can.Update();
        System.out.println("LEDs On!");
    }

    public void LEDset() {
        ld_can.SetColor(4, Color.magenta, 0.9);
        ld_can.SetColor(1, Color.magenta, 0.9);
        ld_can.Update();
        System.out.println("LEDs On!");
    }
}
