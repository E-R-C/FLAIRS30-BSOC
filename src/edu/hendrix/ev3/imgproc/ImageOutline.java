package edu.hendrix.ev3.imgproc;

public interface ImageOutline {
	public int getWidth();
	public int getHeight();

	default public boolean inBounds(int x, int y) {
		return x >= 0 && x < getWidth() & y >= 0 && y < getHeight();
	}
}
