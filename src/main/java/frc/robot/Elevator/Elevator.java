package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.ReefHeight;

public class Elevator extends SubsystemBase{
    public SparkFlex LeftMotor, RightMotor;
    public RelativeEncoder MotionEncoder;
    public SparkClosedLoopController MotionPID;
    private SparkMaxConfig LeftConfig, RightConfig;

    private static Elevator elevator;
    public Shooters shooters = Shooters.getInstance();
    public String NowDoing = "null";

    private Elevator(){
        LeftMotor = new SparkFlex(Constants.LeftID, MotorType.kBrushless);
        RightMotor = new SparkFlex(Constants.RightID, MotorType.kBrushless);
        MotionEncoder = LeftMotor.getEncoder();
        MotionPID = LeftMotor.getClosedLoopController();

        LeftConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.CurrentLimit);
        LeftConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        LeftConfig.softLimit
            .forwardSoftLimit(Constants.MaxHeight)
            .reverseSoftLimit(Constants.MinHeight);
        LeftConfig.closedLoop.apply(Constants.MotionPID);

        RightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(Constants.CurrentLimit);
        RightConfig.softLimit
            .forwardSoftLimit(Constants.MaxHeight)
            .reverseSoftLimit(Constants.MinHeight);
        RightConfig.follow(LeftMotor, true);

        LeftMotor.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command elevate(ReefHeight height) {
        return run(() -> elevate(height.getHeight()))
                .beforeStarting(runOnce(() -> NowDoing = "升到" + height.name()))
                .until(() -> isArrived(height.getHeight()))
                .finallyDo((interrupted) -> NowDoing = "Idle");
    }

    public void elevate(double height){
        MotionPID.setReference(height, ControlType.kPosition);
    }

    public boolean isArrived(double setPoints){
        double currentPosition = MotionEncoder.getPosition();
        return Math.abs(setPoints - currentPosition) <= Constants.ElevatorTolerance;
    }

    public Command seedPosition(){
        return runOnce(() -> {
            MotionEncoder.setPosition(0);
        });
    }

    public static Elevator getInstance(){
        if(elevator == null) elevator = new Elevator();
        return elevator;
    }
}
