package frc.robot.Elevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Millimeters;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

//TODO: 一樣要先改這邊的參數，然後電梯要先空轉測完大概沒問題之後再裝機構上去
public class Constants {
    // 馬達ID
    public static final int LeftID = 21;
    public static final int RightID = 22;
    public static final int CoralShooterID = 23;
    public static final int AlgaeShooterID = 24;

    //電梯的參數
    public static final double ElevatorGearRatio = 10.71;
    public static final double DrumCircumference = Millimeters.of(34.1).times(Math.PI).in(Centimeters); //轉軸直徑
    public static final double PositionConvertionFactor = 1/ElevatorGearRatio*DrumCircumference; //位置轉換因子(編碼器單位轉公分)
    public static final double VelocityConvertionFactor = PositionConvertionFactor/60/60; //速度轉換因子(編碼器單位轉公分/秒) 會除60因為編碼器是RPM
    public static final double MaxHeight = 180; //電梯最大高度，建議用實測的
    public static final double MinHeight = 0; //電梯最低高度,防止馬達跟Encoder方向弄反然後爆掉
    public static final int CurrentLimit = 80; //電流限制
    public static final double ElevatorTolerance = 3; //電梯位置容差,PID調好之後通常會在這個範圍內停下來，在測PID的時候設為0，主要的功能是讓指令可以跳的出來
    public static final ClosedLoopConfig MotionPID = new ClosedLoopConfig() //設定電梯馬達PID
        .pidf(0, 0, 0, 1.0/473)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
}
