package edu.hendrix.ev3.remote;

import java.io.IOException;

import edu.hendrix.ev3.ai.TrainingList;
import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.remote.Mover;
import edu.hendrix.ev3.remote.net.RobotConstants;
import edu.hendrix.ev3.storage.VideoStorage;
import edu.hendrix.ev3.util.Logger;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.video.Video;

// Stores frames in memory and saves all at the end.
public class SimpleVideo {
	
	public static void main(String[] args) {
		new SimpleVideo().start();
	}
	
	public final static int MAX_FRAMES = 700;
	
	private boolean quit = false;
	private byte[] frame;
	private GraphicsLCD g;
	private int framesGrabbed = 0;
	private long startTime;
	private TrainingList examples = new TrainingList();
	
	public void start() {
		try {
			Video wc = setupVideo();
			System.out.println("Video ready!");
			LCD.clear();
			startTime = System.currentTimeMillis();
			Move move = Move.STOP;
			while (move == Move.STOP) {
				move = updatedMove(move);
			}
			System.out.println("Recording...");
			while (!quit && !Button.ESCAPE.isDown() && framesGrabbed < MAX_FRAMES) {
				wc.grabFrame(frame);
				framesGrabbed++;
				Logger.EV3Log.log(String.format("frame %d", framesGrabbed));
				AdaptedYUYVImage image = RobotConstants.setupFrom(frame);
				image.display(g, 0, 0, image.getMeanY());
				examples.add(move, new AdaptedYUYVImage(image));
				move = updatedMove(move);
				Mover.move(move);
			}
			wc.close();
			Logger.EV3Log.log(String.format("%d images grabbed", framesGrabbed));
			LCD.clear();
			Mover.move(Move.STOP);
			System.out.printf("Saving %d images\n", framesGrabbed);
			VideoStorage.getEV3Storage().save(examples);
		} catch (IOException ioe) {
			ioe.printStackTrace();
		} finally {
			quit = true;
			long duration = System.currentTimeMillis() - startTime;
			double fps = 1000.0 * framesGrabbed / duration;
			Logger.EV3Log.log(String.format("Frames: %d\nDuration: %d\nFPS: %5.3f\n", framesGrabbed, duration, fps));
		}
	}
	
	private Video setupVideo() throws IOException {
		g = BrickFinder.getDefault().getGraphicsLCD();
		Video wc = RobotConstants.setupEV3Video();
		frame = wc.createFrame();
		return wc;
	}
	
	private Move updatedMove(Move current) {
		if (Button.UP.isDown()) {
			return Move.FORWARD;
		} else if (Button.LEFT.isDown()) {
			return Move.LEFT;
		} else if (Button.RIGHT.isDown()) {
			return Move.RIGHT;
		} else if (Button.DOWN.isDown()) {
			return Move.BACKWARD;
		} else if (Button.ENTER.isDown()) {
			return Move.STOP;
		} else {
			return current;
		}
	}
}
