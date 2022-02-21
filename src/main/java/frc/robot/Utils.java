package frc.robot;

public class Utils {
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
