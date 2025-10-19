package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

// TODO: 這邊所有東西都要針對你們的實際狀況改
public class Constants {
    public static final int[] LeftIDs = {11,12}; //左邊的馬達的ID
    public static final int[] RightIDs = {13,14}; //右邊的馬達的ID
    public static final double GearRatio = 10.71; //齒輪比
    public static final double WheelCirc = Inches.of(6).times(Math.PI).in(Meters); //輪周長(公尺)
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Centimeters.of(70)); //輪距, Kinematics是用來做運動學計算的
    public static final Pose2d InitialPose = new Pose2d(7.6,7, Rotation2d.kZero); //初始位置

    public static final double PositionConvertionFactor = 1/GearRatio*WheelCirc; //位置轉換因子(編碼器單位轉公尺)
    public static final double VelocityConvertionFactor = PositionConvertionFactor/60/60; //速度轉換因子(編碼器單位轉公尺/秒) 會除兩個60因為編碼器是RPM(第一個是變成速度，第二個是變成每秒鐘)

    public static final int SlipCurrent = 44; //打滑電流限制
    public static final ClosedLoopConfig LeftPID = new ClosedLoopConfig() //設定左馬達PID
        .pidf(0, 0, 0, 1.0/473) //P I D FF, FF=1/Motor kV
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    public static final ClosedLoopConfig RightPID = new ClosedLoopConfig() //設定右馬達PID
        .pidf(0, 0, 0, 1.0/473) //P I D FF, FF=1/Motor kV
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Simulation used
    public static final double RobotMass = 50; //機器人重量(kg)
    public static final double MOI = 1.0/12*RobotMass*2*Math.pow(kinematics.trackWidthMeters, 2);
}
