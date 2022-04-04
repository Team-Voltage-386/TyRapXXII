package frc.robot;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

import edu.wpi.first.wpilibj.DriverStation;

public class Logger {

    public static String[] data = {"","","","","","","","","","","","",""};

    private static Path log = Path.of("sometotalbs");
    private static BufferedWriter os = getDefOs();

    private static BufferedWriter getDefOs() {
        try {
            return Files.newBufferedWriter(log, Charset.defaultCharset(), StandardOpenOption.WRITE);
        } catch (Exception e) {
            
        }
        return null;
    }

    public static void init() {
        Path p = null;
        for (int i = 0; i < 50; i++) {
            Path chk = Path.of("/home/lvuser/logs/", (DriverStation.getEventName()+"_"+DriverStation.getMatchType()+"_"+DriverStation.getMatchNumber()+Integer.toString(i)+".csv"));
            if (!Files.exists(chk)) {
                p = chk;
                break;
            }
        }
        log = p;
        try {
            if (!log.equals(null)) Files.createFile(log);
            os = Files.newBufferedWriter(log, Charset.defaultCharset(), StandardOpenOption.WRITE);
            os.write("Time,Targeting,Shooting,Dist,Heading,X,Y,BC,TX,DrivePower,Hood,Drum,TurnPower\n");
        } catch (IOException e) {
            DriverStation.reportError("LogFileError", e.getStackTrace());
        }
    }

    public static void writeLog() {
        String d = "";
        for (int i = 0; i < data.length; i++) d += data[i]+",";
        d+= "\n";
        //dataBuffer.concat(d);
        try { 
            os.write(d);
            os.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void setBoolean(int i, boolean b) {
        if (b) data[i] = Integer.toString(1);
        else data[i] = Integer.toString(0);
    }

    public static void setDouble(int i, double d) {
        data[i] = Double.toString(d);
    }
    
    public static void setInteger(int i, int d) {
        data[i] = Integer.toString(d);
    }
}
