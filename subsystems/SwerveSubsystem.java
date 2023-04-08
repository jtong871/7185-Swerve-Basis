package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(                            //declaring all four swerve modules, parameters need to be set according to each robot
        0, 0, false, false, 0, 0, false, 0, 0);
    
        private final SwerveModule frontRight = new SwerveModule(
        0, 0, false, false, 0, 0, false, 0, 0);
   
        private final SwerveModule backLeft = new SwerveModule(
        0, 0, false, false, 0, 0, false, 0, 0);
   
        private final SwerveModule backRight = new SwerveModule(
        0, 0, false, false, 0, 0, false, 0, 0);

    private PigeonIMU gyro = new PigeonIMU(0);                    

    public SwerveSubsystem() { 
        new Thread(() -> {                      //this segment simply waits one second before resetting the gyroscope
            try {                               //needs to be done as the gyroscope is still calibrating when initially turned on
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setCompassAngle(0);    
    }
    
    public double getHeading() {
        return Math.IEEEremainder(gyro.getCompassHeading(), 360);           //clamps gyro readings between 0 and 360 degrees, i.e. an angle of 361 degrees is automatically set to 1
    }

    public Rotation2d getRotation2d() {                                     //needed to specify pose on the robot
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {                                                //will constantly update the robot's heading and encoder values on Shuffleboard for debugging
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Front Left Encoder Reading", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Front Right Encoder Reading", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Back Left Encoder Reading", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Back Right Encoder Reading", backRight.getTurningPosition());

        SmartDashboard.putNumber("Front Left Absolute Encoder Reading", frontLeft.getAbsRaw());
        SmartDashboard.putNumber("Front Right Absolute Encoder Reading", frontRight.getAbsRaw());
        SmartDashboard.putNumber("Back Left Absolute Encoder Reading", backLeft.getAbsRaw());
        SmartDashboard.putNumber("Back Right Absolute Encoder Reading", backRight.getAbsRaw());
    }

    public void stopModules() {                                             //call to the stop method of each module stops all the motors
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    

    public void setModuleStates(SwerveModuleState[] desiredStates) {        //sets the states of each module using method setState defined in SwerveModule.java
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.maxSpeed);       //ratios module speeds if a module is asked to go above its maximum speed retain control of the robot
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }
}
