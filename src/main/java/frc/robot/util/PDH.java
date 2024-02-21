package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PDH {

    PowerDistribution PDH;

    public PDH(){
        PDH = new PowerDistribution(1, ModuleType.kRev);
        PDH.setSwitchableChannel(true);
    }

    public void setSwitchableChannel(boolean switchable){
        PDH.setSwitchableChannel(switchable);
    }


    
}
