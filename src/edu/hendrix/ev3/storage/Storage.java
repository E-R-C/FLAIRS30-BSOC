package edu.hendrix.ev3.storage;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Scanner;
import java.util.function.Function;

public class Storage<T> {
	public static final String EV3_BASE = "/home/root";
	public static final String PC_BASE = System.getProperty("user.home") + File.separator + "ev3files";
	
	private String storageDir, defaultName;
	private Function<String,T> extractor;
	
	public Storage(String storageDir, String defaultName, Function<String,T> extractor) {
		this.storageDir = storageDir;
		this.defaultName = defaultName;
		this.extractor = extractor;
	}
	
	public static <T> Storage<T> getEV3Storage(String typeDirName, String typeDefaultName, Function<String,T> extractor) {
		return new Storage<T>("/home/root" + File.separatorChar + typeDirName, typeDefaultName, extractor);
	}
	
	public static <T> Storage<T> getPCStorage(String typeDirName, String typeDefaultName, Function<String,T> extractor) {
		return new Storage<T>(System.getProperty("user.home") + File.separatorChar + typeDirName, typeDefaultName, extractor);
	}
	
	public T open(String choice) throws FileNotFoundException {
		return extractor.apply(altOpen(choice, defaultName));
	}
	
	public T openMostRecent() throws FileNotFoundException {
		String[] picks = choices();
		if (picks.length == 0) {
			throw new FileNotFoundException("No files in storage");
		}
		return open(picks[picks.length - 1]);
	}
	
	public String altOpen(String choice, String filename) throws FileNotFoundException {
		File target = new File(storageDir + choice + File.separator + filename);
		Scanner scanner = new Scanner(target);
		String result = scanner.useDelimiter("\\A").next();
		scanner.close();
		return result;
	}
	
	public String altOpenMostRecent(String filename) throws FileNotFoundException {
		String[] picks = choices();
		return altOpen(picks[picks.length - 1], filename);
	}
	
	public String[] choices(File target) {
		if (target.exists()) {
			String[] targets = target.list();
			sort(targets);
			return targets;
		} else {
			return new String[0];
		}
	}
	
	public String[] choices() {
		return choices(new File(storageDir));
	}
	
	public static void sort(String[] targets) {
		Arrays.sort(targets, (o1,o2) ->  
		   o1.length() < o2.length() ? -1 : o1.length() > o2.length() ? 1 : o1.compareTo(o2));
	}
	
	public void save(T data) throws IOException {
		ensureDirectory();
		File out = new File(getNextFilename());
		output(data.toString(), out);
	}
	
	public void altSave(String rep, String filename) throws IOException {
		String[] picks = choices();
		File out = new File(storageDir + picks[picks.length - 1] + File.separator + filename);
		output(rep, out);
	}
	
	public void update(T data, String choice) throws IOException {
		File target = new File(storageDir + choice);
		File out = new File(getNextFilenameIn(target));
		output(data.toString(), out);
	}
	
	public boolean delete(String choice) {
		return altDelete(choice, defaultName);
	}
	
	public boolean altDelete(String choice, String filename) {
		File target = new File(storageDir + choice + File.separator + filename);
		boolean deleteWorked = target.delete();
		if (deleteWorked) {
			File dir = new File(storageDir + choice);
			if (dir.isDirectory() && dir.list().length == 0) {
				deleteWorked = dir.delete();
			}
		}
		return deleteWorked;
	}
	
	private void output(String contents, File out) throws IOException {
		PrintWriter pw = new PrintWriter(new FileWriter(out));
		pw.println(contents);
		pw.close();
		System.out.println("Data shipped to " + out);
	}
	
	private void ensureDirectory() throws IOException {
		File target = new File(storageDir);
		if (!target.exists()) {
			if (!target.mkdir()) {throw new IOException("Failed to create directory");}
		}
		
		if (!target.isDirectory()) {throw new IOException(storageDir + " is not a directory");}
	}
	
	public String getNextName() {
		return getNextName(new File(storageDir));
	}
	
	public String getNextName(File target) {
		String[] files = choices(target);
		return files.length == 0 ? "1" : Integer.toString(1 + Integer.parseInt(files[files.length - 1]));
	}
	
	private String getFilenamePreview(File target) {
		return target + File.separator + getNextName(target);
	}
	
	private String getNextFilename() {
		File target = new File(storageDir);
		return getNextFilenameIn(target);
	}
	
	private String getNextFilenameIn(File target) {
		String dirname = getFilenamePreview(target);
		File dir = new File(dirname);
		dir.mkdir();
		return dirname + File.separator + defaultName;
	}
}
