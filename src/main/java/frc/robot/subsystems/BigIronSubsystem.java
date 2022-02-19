package frc.robot.subsystems;

import frc.robot.Robot;
import static frc.robot.Constants.ShooterData.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.BigIronConstants.*;

/**Subsystem of the BIG IRON*/
public class BigIronSubsystem extends SubsystemBase {

    //motors
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeID);
    private final CANSparkMax drumLeadMotor = new CANSparkMax(kDrumLeadID, MotorType.kBrushless);
    private final CANSparkMax drumFollowMotor = new CANSparkMax(kDrumFollowID, MotorType.kBrushless);
    private final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoderPin);
    private final TalonSRX beltMotor = new TalonSRX(kBeltID);

    
    //solenoids
    //private final DoubleSolenoid intakeSol1 = new DoubleSolenoid

    //PID Controllers
    private final PIDController pidH = new PIDController(HP,HI,HD);
    private final PIDController pidD = new PIDController(DP,DI,DD);

    //Sensors
    private final DigitalInput breachSensor = new DigitalInput(1);
    private ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kOnboard);

    //Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 0;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public double hoodManPower = 0;

    //Misc.
    private final Timer ejectTimer = new Timer();

    //Private process variables
    private boolean calibrated = false;
    private double hoodSet = 0;
    private boolean drumPIDRunning = false;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    private boolean ballOnTheWay = false;
    private boolean breachSensorFlag = false;
    private boolean intakeSensorFlag = false;

    /**Creates a BigIronSubsystem*/
    public BigIronSubsystem() {
        pidD.reset();
        pidH.reset();

        intakeMotor.configFactoryDefault();
        intakeMotor.configNeutralDeadband(0);

        drumLeadMotor.restoreFactoryDefaults();
        drumFollowMotor.restoreFactoryDefaults();
        drumFollowMotor.follow(drumLeadMotor);

    }

    public void runIntake(boolean b) {
        if (b) intakeMotor.set(ControlMode.PercentOutput, kIntakePower);
        else intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    @Override
    public void periodic() {
        readSensors();
        logic();
        if (!runHoodMan) runHood();
        else {
            hoodMotor.set(ControlMode.PercentOutput, hoodManPower);
        }
        runDrum();
        runFeedBelt();
    }

    private void logic() {
        if (ejectBall) {
            if (breachSensorFlag) ejectTimer.reset();
            else ejectTimer.start();
            if (ejectTimer.hasElapsed(1)) {
                ejectTimer.stop();
                ejectTimer.reset();
                ejectBall = false;
            }
        }
    }

    private void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
    }

    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(pidH.calculate(hoodCurrentPosition), -1*HC, HC);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);//set that hood thing
            else MathUtil.clamp(control,  0, 1);//limit that hood thing
        }
    }

    private void runDrum() {
        if (drumControllerOn) {
            drumPIDRunning = true;
            drumLeadMotor.set(kDrumDirection*pidD.calculate(drumCurrentSpeed, drumSP));
        } else if (drumIdle) {
            drumPIDRunning = true;
            drumLeadMotor.set(kDrumDirection*pidD.calculate(drumCurrentSpeed, kDrumIdleSpeed));
        } else if (ejectBall) {
            drumLeadMotor.set(kDrumDirection*0.4);
        } else {
            drumLeadMotor.set(0);
            if (drumPIDRunning) {
                pidD.reset();
                drumPIDRunning = false;
            }
        }
    }

    private void runFeedBelt() {
        if ((fireTheBigIron && breachSensorFlag && readyToFire()) || ballOnTheWay || ejectBall || runBeltMan) {
            if (breachSensorFlag) ballOnTheWay = false;
            beltMotor.set(ControlMode.PercentOutput, kBeltPower);// run belt
        } else if (!breachSensorFlag && intakeSensorFlag) {
            ballOnTheWay = true;
            beltMotor.set(ControlMode.PercentOutput, kBeltPower);// run belt
        } else beltMotor.set(ControlMode.PercentOutput, 0); //don't run belt
    }

    private final ShuffleboardTab tab = Shuffleboard.getTab("BigIron");
    private final NetworkTableEntry brWidget = tab.add("Breach",false).withPosition(0, 0).getEntry();
    private final NetworkTableEntry inWidget = tab.add("intake",false).withPosition(0, 1).getEntry();
    private final NetworkTableEntry encWidget = tab.add("HEncoder",0).withPosition(0, 2).getEntry();
    private final NetworkTableEntry botwWidget = tab.add("botw",false).withPosition(1, 0).getEntry();

    private void updateWidgets() {
        brWidget.setBoolean(breachSensorFlag);
        inWidget.setBoolean(intakeSensorFlag);
        encWidget.setDouble(hoodCurrentPosition);
        botwWidget.setBoolean(ballOnTheWay);
    }
}