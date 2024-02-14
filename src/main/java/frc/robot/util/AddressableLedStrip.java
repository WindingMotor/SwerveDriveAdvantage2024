
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
            //everyOther(Color.kRed, Color.kBlue);
            everyOther(Color.kGreen, Color.kBlue);
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
        for(var i = 0; i < length; i++){
            int decay = MathCalc.random(0, ((i+1) * 10) / length + 1);
            ledBuffer.setRGB(i, 160 - decay, 120 - decay, 90 - decay);
        }
    }
}
