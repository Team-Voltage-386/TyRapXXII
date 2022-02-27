package frc.robot;

public class Utils {

    public static class Flags {
        public static boolean hoopTargeted = false;
        public static boolean hoopLocked = false;
        public static double targetDistance = 9999;
        public static boolean complianceOverride = false;
    }

    /**Lerp
     * @param a first number
     * @param b second number
     * @param factor lerp factor
    */
    public static double lerp(double a, double b, double factor) {
        double c = b - a;
        c = factor*c;
        return a+c;
    }
}
