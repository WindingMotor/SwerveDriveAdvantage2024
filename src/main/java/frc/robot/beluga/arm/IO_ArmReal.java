
package frc.robot.beluga.arm;
import com.revrobotics.*;
import frc.robot.wmlib2j.util.Builder;


public class IO_ArmReal implements IO_ArmBase {

    private CANSparkMax motorOne;       
    private CANSparkMax motorTwo;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    private SparkPIDController motorOnePID;
    private SparkPIDController motorTwoPID;

    private double setpointPosition;

    public IO_ArmReal(){

        motorOne = Builder.createNeo(21, false, 30);
        motorTwo = Builder.createNeo(22, false, 30);

        motorOneEncoder = Builder.createEncoder(motorOne, 0.0105, 0.0105);
        motorTwoEncoder = Builder.createEncoder(motorTwo, 0.0105, 0.0105);

        motorOnePID = motorOne.getPIDController();
        motorTwoPID = motorTwo.getPIDController();
        
    }
    



}
