package edu.hendrix.ev3.storage;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.EnumSet;
import java.util.Scanner;
import java.util.concurrent.BlockingQueue;
import java.util.function.Consumer;

import edu.hendrix.ev3.ai.TrainingList;
import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.util.Duple;

public class VideoStorage {
	
	private final static String VIDEO_DIR_NAME = "ev3Video";
	
	private File baseDir, currentDir;
	private boolean firstImageSaved = false;
	
	private VideoStorage(String where) {
		baseDir = new File(where);
		if (baseDir.exists()) {
			if (!baseDir.isDirectory()) {
				throw new IllegalStateException(where + " exists, but is not a directory");
			}
		} else {
			boolean made = baseDir.mkdir();
			if (!made) {throw new IllegalStateException("Failed to create directory " + where);}
		}
	}
	
	public static VideoStorage getEV3Storage() {
		return new VideoStorage(Storage.EV3_BASE + "/" + VIDEO_DIR_NAME);
	}
	
	public static VideoStorage getPCStorage() {
		return new VideoStorage(Storage.PC_BASE + File.separator + VIDEO_DIR_NAME);
	}
	
	private void makeNewDir() {
		String newName = getNewName();
		currentDir = getDirFor(newName);
		currentDir.mkdir();	
	}
	
	private String getNewName() {
		String[] files = getVideoChoices();
		return files.length == 0 ? "1" : Integer.toString(1 + Integer.parseInt(files[files.length - 1]));
	}
	
	public void save(TrainingList examples) throws FileNotFoundException {
		makeNewDir();
		for (int i = 0; i < examples.size(); i++) {
			saveImg(i, examples.get(i));
		}
	}
	
	public void save(int id, Move m, AdaptedYUYVImage img) throws FileNotFoundException {
		if (!firstImageSaved) {
			firstImageSaved = true;
			makeNewDir();
		}
		saveImg(id, m, img);
	}
	
	private void saveImg(int id, Duple<Move,AdaptedYUYVImage> example) throws FileNotFoundException {
		saveImg(id, example.getFirst(), example.getSecond());
	}
	
	private void saveImg(int id, Move m, AdaptedYUYVImage img) throws FileNotFoundException {
		File imgFile = new File(currentDir, Integer.toString(id));
		PrintWriter out = new PrintWriter(imgFile);
		out.printf("%s|%s\n", m.toString(), img.toString());
		out.close();
	}
	
	public String[] getVideoChoices() {
		String[] choices = baseDir.list();
		Storage.sort(choices);
		return choices;
	}
	
	private File getDirFor(String choice) {
		return new File(baseDir.getAbsolutePath() + File.separator + choice);
	}
	
	public int getNumFramesFor(String choice) {
		return getDirFor(choice).list().length;
	}
	
	public TrainingList open(String choice, EnumSet<Move> permittedMoves) throws FileNotFoundException {
		File dir = getDirFor(choice);
		TrainingList result = new TrainingList();
		storeConditionallyFrom(dir, getFrameFiles(dir, choice), permittedMoves, from -> result.add(from));
		return result;
	} 
	
	public TrainingList open(String choice) throws FileNotFoundException {
		return open(choice, EnumSet.allOf(Move.class));
	}
	
	public void threadedOpen(String choice, EnumSet<Move> permittedMoves, BlockingQueue<Duple<Move,AdaptedYUYVImage>> imgs) throws FileNotFoundException {
		File dir = getDirFor(choice);
		storeConditionallyFrom(dir, getFrameFiles(dir, choice), permittedMoves, from -> {try {imgs.put(from);}catch(InterruptedException e){System.out.println("what?");}});		
		try {
			imgs.put(new Duple<>(Move.NONE, null));
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private String[] getFrameFiles(File dir ,String choice) {
		String[] frameFiles = dir.list();
		Storage.sort(frameFiles);
		return frameFiles;
	}
	
	private void storeConditionallyFrom(File dir, String[] frameFiles, EnumSet<Move> permittedMoves, Consumer<Duple<Move,AdaptedYUYVImage>> storer) throws FileNotFoundException {
		for (String frameFile: frameFiles) {
			Duple<Move,AdaptedYUYVImage> from = getImageFrom(new File(dir, frameFile));
			if (permittedMoves.contains(from.getFirst())) {
				storer.accept(from);
			}
		}
	}
	
	private Duple<Move,AdaptedYUYVImage> getImageFrom(File file) throws FileNotFoundException {
		Scanner s = new Scanner(file);
		String[] parts = s.nextLine().split("\\|");
		s.close();
		Move m = Move.valueOf(parts[0]);
		return new Duple<>(m,AdaptedYUYVImage.fromString(parts[1]));
	}
}
