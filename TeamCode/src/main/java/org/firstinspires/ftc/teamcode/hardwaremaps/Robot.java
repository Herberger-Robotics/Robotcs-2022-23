package org.firstinspires.ftc.teamcode.hardwaremaps;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.hardwaremaps.HardwareWrappers.CoolMotor;
//import org.firstinspires.ftc.teamcode.subsystems.DuckWheel.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.LiftArm.LiftArm;

import java.util.List;

public class Robot {

        // Static variable reference of single_instance
        // of type Singleton
        private static Robot single_instance = null;

        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();

        public BNO055IMU imu;

        //drivetrain
        public CoolMotor rightFront = null;
        public CoolMotor leftFront = null;
        public CoolMotor rightBack = null;
        public CoolMotor leftBack = null;

        public Motor duckMotor = null;

        public CoolMotor lift = null;
        public CoolMotor turnTable = null;

        //public Camera camera = null;

        public LiftArm liftArm = null;
        public DriveTrain driveTrain = null;
        //public DuckWheel duckWheel = null;
        private static RunType lastRan = RunType.MANUAL;
        public static RunType setLastRan(RunType toSet) {
            lastRan = toSet;
            return lastRan;
        }
        public static RunType getLastRan() {
            return lastRan;
        }

        public enum RunType {
            AUTONOMOUS,
            MANUAL
        }



    // Constructor
        // Here we will be creating private constructor
        // restricted to this class itself
        private Robot()
        {

        }

        // Static method
        // Static method to create instance of Singleton class
        public static Robot getInstance()
        {
            if (single_instance == null)
                single_instance = new Robot();

            return single_instance;
        }
        public static Robot resetInstance()
        {
                single_instance = new Robot();
                return single_instance;
        }

        public void init(HardwareMap ahwMap, DriveTrain.DriveMode driveMode,boolean initIMU)
        {
            CommandScheduler.getInstance().reset();
            hwMap = ahwMap;
            //camera = new Camera(hwMap);
            driveTrain = new DriveTrain(hwMap, driveMode);
            liftArm = new LiftArm(hwMap);
           // duckWheel = new DuckWheel(hwMap);

            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

            if(initIMU) {
                imu = hwMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imu.initialize(parameters);
            }
        }

        public void clearBulkCache() {
            List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

            for(LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }

}
