package frc.robot.Elevator;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.ReefHeight;

public class Shooters extends SubsystemBase{
    public SparkMax CoralShooter, AlgaeShooter;
    private SparkMaxConfig CoralConfig, AlgaeConfig;

    private static Shooters shooters;

    private Shooters(){
        CoralShooter = new SparkMax(Constants.CoralShooterID, MotorType.kBrushed);
        AlgaeShooter = new SparkMax(Constants.AlgaeShooterID, MotorType.kBrushed);

        CoralConfig = new SparkMaxConfig();
        AlgaeConfig = new SparkMaxConfig();

        CoralConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(20);
        AlgaeConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(20);
        
        CoralShooter.configure(CoralConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        AlgaeShooter.configure(AlgaeConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command shootCoral(boolean isIntake){
        return runEnd(() -> {
            CoralShooter.set(0.8);
            Elevator.getInstance().NowDoing = "Shooting Coral";
        }, () -> {
            CoralShooter.stopMotor();
            Elevator.getInstance().NowDoing = "Idle";
        }).beforeStarting(Elevator.getInstance().elevate(ReefHeight.L0)).withTimeout(Seconds.of(isIntake ? 125 : 0.3));
    }

    public Command shootAlgae(boolean isIntake){
        return runEnd(() -> {
            AlgaeShooter.set((isIntake ? 1 : -1)*0.5);
            Elevator.getInstance().NowDoing = "Shooting Algae";
        }, () -> {
            AlgaeShooter.stopMotor();
            Elevator.getInstance().NowDoing = "Idle";
        });
    }

    public static Shooters getInstance(){
        if(shooters == null) shooters = new Shooters();
        
        return shooters;
    }
}
