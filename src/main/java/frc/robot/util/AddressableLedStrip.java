
package frc.robot.util;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLedStrip extends SubsystemBase{

    private final AddressableLED ledStrip;
    private final AddressableLEDBuffer ledBuffer;
    private final int length;

    public enum LEDState{
        RAINBOW, GREEN, BLUE, RED, PINK, WHITE, ORANGE, FLAME, ALLIANCE
    }

    private LEDState state;
    private int hue;

    public AddressableLedStrip(int port, int length){

        this.length = length;

        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(ledBuffer.getLength());

        ledStrip.setData(ledBuffer);

        state = LEDState.RAINBOW;

        hue = 0;

        ledStrip.start();
    }

    @Override
    public void periodic(){

        if(DriverStation.isDisabled()){
            //solid(Color.kPurple);
            everyOther(Color.kRed, Color.kGreen);
            //everyOther(Color.kGreen, Color.kBlue);
        }else{

        switch(state){
            case RAINBOW:
                rainbow();
                break;

            case GREEN:
                solid(Color.kGreen);
                break;

            case BLUE:
                solid(Color.kGreen);
                break;

            case RED:
                solid(Color.kRed);
                break;
            case WHITE:
                solid(Color.kWhite);
                break;

            case ORANGE:
                solid(Color.kOrange);
                break;

            case FLAME:
                flame();
                break;

            case ALLIANCE:
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    if(alliance.get() == Alliance.Red){
                        solid(Color.kRed);
                    }else if(alliance.get() == Alliance.Blue){
                        solid(Color.kBlue);
                    }
                }else{
                    solid(Color.kOrange);
                }
                break;
        }
    }

        ledStrip.setData(ledBuffer);

    }

    public void setState(LEDState state){
        this.state = state;
    }

    private void rainbow(){
        for(var i = 0; i < length; i++){
            final var hue = (this.hue + (i * 180 / length)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        this.hue += 3;
        this.hue %= 180;
    }

    private void solid(Color color){
        for(var i = 0; i < length; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    private void everyOther(Color color1, Color color2){
        for(var i = 0; i < length; i++){
            if(i % 2 == 0){
                ledBuffer.setLED(i, color1);
            }else{
                ledBuffer.setLED(i, color2);
            }
        }
    }

    private void everyOther(int sectionLength, Color color1, Color color2){
        int segmentStart = 0;
        while(segmentStart < length){
            for(var i = segmentStart; i < Math.min(length, segmentStart + sectionLength); i++){
                ledBuffer.setLED(i, color1);
            }
            for(var i = segmentStart + sectionLength; i < Math.min(length, segmentStart + 2 * sectionLength); i++){
                ledBuffer.setLED(i, color2);
            }
            segmentStart += 2 * sectionLength;
        }
    }

    private void flame(){
        byte[] heat = new byte[length];
        final int COOLING =  55;
        final int SPARKING =  120;
    
        // Step  1. Cool down every cell a little
        for(var i =  0; i < length; i++){
            heat[i] = (byte)Math.max(0, heat[i] - random(0, ((COOLING *  10) / length) +  2));
        }
    
        // Step  2. Heat from each cell drifts 'up' and diffuses a little
        for(var k = length -  3; k >  0; k--){
            heat[k] = (byte)((heat[k -  1] + heat[k -  2] + heat[k -  2]) /  3);
        }
    
        // Step  3. Randomly ignite new 'sparks' of heat near the bottom
        if(random(0,  255) < SPARKING){
            int y = random(0,  7);
            heat[y] = (byte)(heat[y] + random(160,  255));
        }
    
        // Step  4. Map from heat cells to LED colors
        for (var j =  0; j < length; j++) {
            byte colorIndex = (byte)scale8(heat[j],  240);
            // Use the color index to map to a color palette
            // You would need to define a color palette similar to the one in Source  1
            Color color = paletteLookup(colorIndex); // Replace with actual implementation
            ledBuffer.setLED(j, color);
        }
    }
    
    private int random(int min, int max){
        return (int)(Math.random() * (max - min +  1)) + min;
    }

    private int scale8(int num, int ceilVal){
        return (num * ceilVal) >>  8;
    }

    private static final Color[] palette = {
        new Color(0,  0,  0), // Black
        new Color(128,  0,  0), // Dark red
        new Color(255,  0,  0), // Red
        new Color(255,  165,  0), // Orange
        new Color(255,  255,  0), // Yellow
        new Color(255,  255,  255) // White
    };
    
    private Color paletteLookup(int index){
        // Ensure index is within bounds
        index = Math.max(0, Math.min(index, palette.length -  1));
        // Return the corresponding color from the palette
        return palette[index];
    }
}
