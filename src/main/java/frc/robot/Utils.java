package frc.robot;

public class Utils {

    public static class Flags {
        public static boolean hoopTargeted = false;
        public static boolean hoopLocked = false;
        public static double targetDistance = 9999;
        public static boolean complianceOverride = false;
        public static boolean hoopVisible = false;
    }

    /**Lerp
     * @param a first number
     * @param b second number
     * @param factor lerp factor
     */
    public static double lerpA(double a, double b, double factor) {
        double c = b-a;
        c = factor * c;
        return a + c;
    }

    public static double lerpB(double a, double b, double factor) {
        double c = Math.abs(b - a);
        c = factor * c;
        return a + c;
    }

    public static String ourAlliance = "";
    public static String antiAlliance = "";

    public static String giveAntiAlliance(String ourAlliance) {
        String output = "";
        if (ourAlliance.equals("Blue"))
            output = "Red";
        else if (ourAlliance.equals("Red"))
            output = "Blue";
        else
            output = "Invalid";
        return output;
    }
}
