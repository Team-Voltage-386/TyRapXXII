package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Constants.ShooterData;
import frc.robot.Utils.Flags;

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import static frc.robot.Constants.BigIronConstants.*;

/**
 * Subsystem of the BIG IRON
 * (Ball movement stuff)
 */
public class BigIronSubsystem extends SubsystemBase {
    // motors
    private final CANSparkMax intakeMotor = new CANSparkMax(kIntakeID, MotorType.kBrushless);
    private final CANSparkMax drumOneMotor = new CANSparkMax(kDrumOneID, MotorType.kBrushless);
    private final CANSparkMax drumTwoMotor = new CANSparkMax(kDrumTwoID, MotorType.kBrushless);
    private final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoderPin);
    private final TalonSRX beltMotor = new TalonSRX(kBeltID);

    // solenoids
    private final DoubleSolenoid intakeForward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
            kChannelIntakeForwardGo, kChannelIntakeForwardVent);
    private final DoubleSolenoid intakeBackward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
            kChannelIntakeBackwardGo, kChannelIntakeBackwardVent);
    private final static Value kGo = Value.kForward;
    private final static Value kVent = Value.kReverse;

    // PID Controllers
    private final PIDController pidH = new PIDController(HP, HI, HD);
    public final PIDController pidD = new PIDController(DP, DI, DD);

    // Sensors
    private final DigitalInput breachSensor = new DigitalInput(kBreachSensorPin);
    private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final AnalogInput pressureSensor = new AnalogInput(3);// airTank sensor
    private final DigitalInput hoodLimit = new DigitalInput(kHoodDownLimitPin);

    double tankPressure;

    // Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 2000;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public String ball1Col = "null";
    public String ball2Col = "null";
    public double hoodManPower = 0;
    public int ballCount = 0;
    public String colorOurs;
    public String colorAnti;

    // Misc.
    private final Timer ejectTimer = new Timer();
    private final Timer ledTimer = new Timer();

    // Private process variables
    private boolean calibrated = false;
    public double hoodSet = 0.05;
    private boolean drumPIDRunning = false;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    public boolean ballOnTheWay = false;
    public boolean breachSensorFlag = false;
    private boolean intakeSensorFlag = false;

    /** Creates a BigIronSubsystem */
    public BigIronSubsystem() {
        pidD.reset();
        pidH.reset();
        hoodMotor.configNeutralDeadband(0);

        drumOneMotor.setInverted(false);
        drumTwoMotor.setInverted(false);
        intakeOut = false;

        //drumLeadMotor.restoreFactoryDefaults();
        //drumFollowMotor.restoreFactoryDefaults();
        drumTwoMotor.follow(drumOneMotor,true);
        ledTimer.start();
    }

    public void ballFailedDebug() {
        if (ballCount == 0) {
            ballOnTheWay = !ballOnTheWay;
            woundBack = false;
        } else if (ballCount == 1) {
            ballCount = 0;
            ballOnTheWay = false;
            woundBack = false;
            ejectBall = false;
            ef = false;
            eff = false;
        } else if (ballCount == 2) {
            ballCount = 1;
            woundBack = false;
        }
    }

    public void reset() {
        pidD.reset();
        pidH.reset();
        beltTimer.stop();
        beltTimer.reset();
        ejectTimer.stop();
        ejectTimer.reset();
        ejectBall = false;
        calibrated = false;
        //drumIdle = false;
        ball1Col = "null";
        ball2Col = "null";
        ballOnTheWay = false;
        ballCount = 0;
        intakeSensorFlag = false;
        breachSensorFlag = false;
        fireTheBigIron = false;
        ef = false;
        eff = false;
    }

    public void runIntake(boolean b) {
        if (intakeOut && b) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    public boolean intakeOut = true;
    private Timer t = new Timer();

    /**
     * Do the intake
     * 
     */
    public void intakeDo(boolean b) {
        if (b) {
            if (!intakeOut) {
                intakeOut = true;
                t.start();
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeOut = false;
                t.stop();
                t.reset();
                intakeForward.set(kVent);
                intakeBackward.set(kGo);
            }
        } else if (t.hasElapsed(1)) {
            if (Flags.complianceOverride) {
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeForward.set(kVent);
                intakeBackward.set(kVent);
            }
        }
        if (intakeOut) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    public void reLoad() {
        ball1Col = ball2Col;
        if (ballCount > 1) {
            ball2Col = "null";
            ballOnTheWay = true;
        }
        ballCount = 0;
    }

    @Override
    public void periodic() {
        readSensors();
        logic();
        runHood();
        runDrum();
        runFeedBelt();
        updateWidgets();
        //setLED();
    }

    boolean ef = false;
    boolean eff = false;

    private void logic() {
        if (ejectBall) {
            if (!ef) {
                ejectTimer.reset();
                ejectTimer.start();
                ef = true;
                eff = false;
            }
        }
    }

    private void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
        breachSensorFlag = !breachSensor.get();
        double inSens = intakeSensor.getProximity();
        intakeSensorFlag = (inSens > 310 || inSens < 230) && intakeOut;
        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        tankPressure = 250 * (pressureSensor.getVoltage() / 5) - 25;
        hoodLowLimit = !hoodLimit.get();
        drumCurrentSpeed = -1*drumOneMotor.getEncoder().getVelocity();
    }

    private void runHood() {
        if (calibrated) {
            double control = MathUtil.clamp(pidH.calculate(hoodCurrentPosition, hoodSet), -1, 1);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else {
                hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, 0, 1));// limit that hood thing
                pidH.reset();
                hoodEncoder.reset();
            }
        } else {
            if (hoodLowLimit) {
                calibrated = true;
                pidH.reset();
                hoodEncoder.reset();
            }
            else {
                hoodMotor.set(ControlMode.PercentOutput, -0.9);
            }
        }
        //hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    private void runDrum() {
        if (fireTheBigIron || drumIdle) {
            double control = kDrumDirection * pidD.calculate(drumCurrentSpeed, drumSP);
            drumOneMotor.set(control);
        } else if (ejectBall || eff) {
            drumOneMotor.set(kDrumDirection*0.3);
        } else drumOneMotor.set(0);
    }

    private Timer beltTimer = new Timer();
    private boolean woundBack = false;

    private void runFeedBelt() {
        if (fireTheBigIron) {
            if (readyToFire() && Flags.hoopLocked) {
                beltMotor.set(ControlMode.PercentOutput, kBeltPower); 
            } else beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 0) {
            if (!ballOnTheWay) {
                if (!breachSensorFlag && intakeSensorFlag) {
                    ballOnTheWay = true;
                    beltTimer.start();
                    if (ball1Col.equals("null")) ball1Col = getColor();
                }
                beltMotor.set(ControlMode.PercentOutput, 0);
            } else {
                runIntake(true);
                beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                if (breachSensorFlag || beltTimer.hasElapsed(5)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    ballOnTheWay = false;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    if (breachSensorFlag) {
                        woundBack = false;
                        ballCount++;
                    }
                }
            }
        } else if (ballCount == 1) {
            if (!woundBack) {
                if (breachSensorFlag) beltTimer.start();
                beltMotor.set(ControlMode.PercentOutput, kBeltReversePower);
                if (beltTimer.hasElapsed(0.6)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    woundBack = true;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                }
            } else {
                if (intakeSensorFlag) {
                    ballCount++;
                } else if (ejectBall) {
                    if (breachSensorFlag) {
                        if (ejectTimer.hasElapsed(0.5)) {
                            beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                            eff = true;
                            ejectBall = false;
                        }
                        else beltMotor.set(ControlMode.PercentOutput, 0);
                    } else beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                } else if (eff) {
                    if (!breachSensorFlag) {
                        beltMotor.set(ControlMode.PercentOutput, 0);
                        reLoad();
                        ef = false;
                        eff = ef;
                    }
                } else beltMotor.set(ControlMode.PercentOutput, 0); 
            }
        } else if (ballCount == 2) {
            if (ball2Col.equals("null")) ball2Col = getColor();
            else if (ejectBall) {
                if (ejectTimer.hasElapsed(0.5)) {
                    beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                    ejectBall = false;
                    eff = true;
                } else beltMotor.set(ControlMode.PercentOutput, 0);
            } else if (eff) {
                if (!breachSensorFlag) {
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    ball1Col = ball2Col;
                    ball2Col = "null";
                    ballCount = 1;
                    ef = false;
                    eff = ef;
                    fireTheBigIron = false;
                    ejectBall = true;
                    woundBack = true;
                }
            } else beltMotor.set(ControlMode.PercentOutput, 0);  
            if (!breachSensorFlag) beltMotor.set(ControlMode.PercentOutput, kBeltPower);
    } 
}

    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private final NetworkTableEntry mainDist = mainTab.add("dist",0).withPosition(5,0).withSize(1, 1).getEntry();
    private final NetworkTableEntry mainBC = mainTab.add("BallCount",0).withPosition(3,0).withSize(1,1).getEntry();

    private void updateWidgets() {
        mainDist.setDouble(Utils.Flags.targetDistance);
        mainBC.setDouble(ballCount);
    }

    private String getColor() {
        double b = intakeSensor.getBlue();
        double r = intakeSensor.getRed();
        if (b > r) return "Blue";
        return "Red";
    }

    public void setAimDistance(double m) {
        int i = ShooterData.distances.length-1;
        for (int j = 1; j < ShooterData.distances.length; j++) {
            if (m < ShooterData.distances[j]) {
                i = j;
                break;
            }
        }
        double upper = ShooterData.distances[i];
        double lower = ShooterData.distances[i-1];
        double lerpFactor = (m-lower)/Math.abs(upper-lower);
        upper = ShooterData.drumSpeeds[i];
        lower = ShooterData.drumSpeeds[i-1];
        drumSP = (int)Utils.lerpB(lower, upper, lerpFactor);
        upper = ShooterData.hoodPositions[i];
        lower = ShooterData.hoodPositions[i-1];
        hoodSet = Utils.lerpB(lower, upper, lerpFactor);
    }


/*

    ///LED CRAP
    AddressableLED ledA = new AddressableLED(16);

    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(16);

    Color blue = new Color(0, 0, 200);
    Color red = new Color(200,0,0);
    Color yellow = new Color(119,105,1);
    int lastBallCount = 0;
    int aniProg = 0;
    private Color col = null;
    boolean aniRunning = false;

    private void setLED() {
        if (col == null) {
            if (DriverStation.getAlliance().toString() == "Blue") col = blue;
            else col = red;
        }
        if (ledTimer.advanceIfElapsed(0.5)){
            for (int i = 0; i < 8; i++) {
                if (ballCount == 2 && i < aniProg) ledBuffer.setLED(i+8, col);
                else ledBuffer.setLED(i+8, yellow);
                if (ballCount > 0 && i < aniProg) ledBuffer.setLED(i, col);
                else ledBuffer.setLED(i, yellow);
            }
            ledA.setData(ledBuffer);
            boolean newBall = ballCount > lastBallCount;
            if (aniRunning) {
                if (aniProg > 7) {
                    aniProg = 0;
                    aniRunning = false;
                } else aniProg++;
            }
            if (newBall) aniRunning = true;
        }
        lastBallCount = ballCount;
    }*/
}