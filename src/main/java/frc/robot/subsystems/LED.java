package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private final int id;
    private final int length;
    private AddressableLED led;
    private AddressableLEDBuffer buffer;


    public LED(int id, int length) {

        this.id = id;
        this.length = length;
        led = new AddressableLED(id);
        led.setLength(length);
        buffer = new AddressableLEDBuffer(length);
        led.start();

    }

    public void setLED(int index, int r, int g, int b) {
        
        buffer.setRGB(index, r, g, b);

    }

    public int getLength() {

        return this.length;

    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }
    
}
