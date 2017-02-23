package edu.hendrix.ev3.remote.net;

import java.io.IOException;

import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import lejos.hardware.BrickFinder;
import lejos.hardware.video.Video;

public class RobotConstants {
	public final static int WIDTH = 160, HEIGHT = 120, IMAGE_BYTES = WIDTH * HEIGHT * 2;
	
	public final static float MAX_SONAR_DISTANCE = 2.5f;
	
	public final static int MSG_PORT = 8001, IMAGE_PORT = MSG_PORT + 2;
	
	public static final byte[] addr = new byte[]{10,0,1,1};
	
	public static AdaptedYUYVImage setupFrom(byte[] pixels) {
		if (pixels.length != IMAGE_BYTES) {
			throw new IllegalArgumentException("Wrong number of pixels: " + pixels.length);
		}
		return new AdaptedYUYVImage(pixels, WIDTH, HEIGHT);
	}
	
	public static AdaptedYUYVImage makeBlank() {
		return setupFrom(new byte[IMAGE_BYTES]);
	}
	
	public static AdaptedYUYVImage makeRandomized() {
		byte[] pixels = new byte[IMAGE_BYTES];
		int range = Byte.MAX_VALUE - Byte.MIN_VALUE;
		for (int i = 0; i < pixels.length; i++) {
			pixels[i] = (byte)(Math.random() * range + Byte.MIN_VALUE);
		}
		return setupFrom(pixels);
	}
	
	public static Video setupEV3Video() throws IOException {
		Video wc = BrickFinder.getDefault().getVideo();
		wc.open(WIDTH,HEIGHT);
		return wc;
	}
}
