package frc.robot.util;

import java.text.SimpleDateFormat;
import java.util.Date;

public class Logger {

    private static String tag;

    public Logger(String name, Integer year) {

        tag = name + year;
        
    }

    /** Used to log Robot messages to console.
     * @param message any string to be displayed in console. **/
    public void log (String... message) {
        for (String out : message) System.out.println("[" + new SimpleDateFormat("HH:mm:ss").format(new Date()) + "] [" + tag + "] " + out);
    }

    /** Used to log Robot errors to console.
     * @param message any string to be displayed in console. **/
    public void error (String... message) {
        for (String out : message) System.err.println("[" + new SimpleDateFormat("HH:mm:ss").format(new Date()) + "] [" + tag + "] " + out); 
    }



}