package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    

    private final WPI_TalonFX driveMotor;           //declaration of all motors, encoders, PID Controllers and module specifications
    private final WPI_TalonSRX turnMotor;           //abs Encoder is Analog, which has to be programmed under the AnalogEncoder class instead of the standard Encoder class

    private final Encoder driveEnc;
    private final Encoder turnEnc;
    private final AnalogEncoder turnEncAbs;

    private final PIDController turnPID;
    private final boolean isReversed;               //parameter specifying whether the abs Encoder is reversed or not
    private final double offset;                    //abs Encoder will not always read what the actual wheel position is, this variable is meant to account for that



    public SwerveModule(                            
        int driveId, int turnId, boolean driveReversed, boolean turnReversed, int absId, double absOffset, boolean absReversed, int driveEncId, int turnEncId
    ) {
        driveMotor = new WPI_TalonFX(driveId);
        turnMotor = new WPI_TalonSRX(turnId);
        driveMotor.setInverted(driveReversed);
        turnMotor.setInverted(turnReversed);

        turnEncAbs = new AnalogEncoder(absId);
        offset = absOffset;
        isReversed = absReversed;

        driveEnc = new Encoder(driveEncId, driveEncId + 1);
        turnEnc = new Encoder(turnEncId, turnEncId + 1);

        turnPID = new PIDController(ModuleConstants.kP, 0, 0);              //kP needs to be tuned according to each robot, kI and kD are optional for swerve programming
        turnPID.enableContinuousInput(-Math.PI, Math.PI);                           //tells the PID Controller that PID values can pass through the given values, jumping from pi to negative pi
        
        driveEnc.setDistancePerPulse(ChassisConstants.driveGearRatio * Math.PI * ChassisConstants.wheelDiameter);       //converts encoder ticks to radians, needs to be tweaked according to chassis and modules
        turnEnc.setDistancePerPulse(ChassisConstants.turnGearRatio * 2 * Math.PI);
        turnEncAbs.setDistancePerRotation(ChassisConstants.turnGearRatio * 2 * Math.PI);
    }

    public double getDrivePosition() {
        return driveEnc.getDistance();
    }

    public double getTurningPosition() {
        return turnEnc.getDistance() % (2.0 * Math.PI);
    }

    public double getAbsPos() {
        double angle = turnEncAbs.getAbsolutePosition() % (2.0 * Math.PI);
        angle -= offset;
        return angle * (isReversed ? -1.0 : 1.0);
    }

    public double getAbsRaw() {                     //used for finding abs Encoder offset
        double angle = turnEncAbs.getAbsolutePosition() % (2.0 * Math.PI);
        return angle;
    }

    public void resetEncoders() {
        driveEnc.reset();
        turnEnc.reset();        //turnEnc needs to inherit abs reading somehow
    }

    public double getTurnRate() {
        return turnEnc.getRate();
    }

    public double getDriveVelocity() {
        return driveEnc.getRate();
    }

    public SwerveModuleState getState() {                                           //returns the current speed and angle of the specified module
        return new SwerveModuleState(getDriveVelocity(), getState().angle);         
    }
    
    public void setState(SwerveModuleState state) {                                 //allows inputs made by WPI libraries to set motors to specified outputs
        if(Math.abs(state.speedMetersPerSecond) < 0.005) {
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);                         //prevents the need for each module to turn more than 90 degrees at a time
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.maxSpeed);               //ex. module has to turn 135 degrees, optimize lets module turn 45 degrees in the opposite direction and reverse motors
        turnMotor.set(turnPID.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {            //does as expected
        driveMotor.set(0);
        turnMotor.set(0);
    }

   
}
