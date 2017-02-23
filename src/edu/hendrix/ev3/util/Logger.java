package edu.hendrix.ev3.util;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class Logger {
	private File logFile;
	private long timestamp;
	
	public Logger(String prefix) {
		String logFileName = System.getProperty("sun.java.command");
		logFileName = logFileName.substring(logFileName.lastIndexOf('.') + 1) + ".log";
		logFile = new File(prefix + logFileName);
		if (logFile.exists()) {
			logFile.delete();
		}
	}
	
	public String getFilename() {return logFile.toString();}
	
	public static Logger EV3Log = new Logger("/home/lejos/programs/");
	public static Logger ClientLog = new Logger("/Users/gabriel/");
	
	public void markTime() {
		timestamp = System.currentTimeMillis();
		format("Timestamp %d", timestamp);
	}
	
	public void recordElapsedTime(String label) {
		long now = System.currentTimeMillis();
		long elapsed = now - timestamp;
		timestamp = now;
		format("%s: elapsed %d", label, elapsed);
	}
	
	public synchronized void log(String line) {
		try {
			PrintWriter pw = new PrintWriter(new FileWriter(logFile, logFile.exists()));
			pw.println(line);
			pw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void format(String format, Object... args) {
		log(String.format(format, args));
	}
}
