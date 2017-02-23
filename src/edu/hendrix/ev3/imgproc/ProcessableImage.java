package edu.hendrix.ev3.imgproc;

import javafx.scene.paint.Color;

public interface ProcessableImage<T extends ProcessableImage<T>> extends ImageOutline {
	public int getIntensity(int x, int y);
	public void setRGB(int x, int y, Color c);
	
	default public boolean canShrinkBy(int shrinkFactor) {
		return getWidth() % shrinkFactor == 0 && getHeight() % shrinkFactor == 0;
	}
	public T shrunken(int shrink);
	
	default public T twoPassConvolve(int[] kernel1, int[] kernel2) {
		T pass1 = xConvolve1D(kernel1);
		return pass1.yConvolve1D(kernel2);
	}
	public T xConvolve1D(final int[] kernel);
	public T yConvolve1D(final int[] kernel);
	
	// Python code for calculation:
	// def gaussian(x, sigma):
	//     return 1.0/math.sqrt(2*math.pi*sigma**2) * math.exp(-x**2/(2*sigma**2))
	// >>> gaussian(0, 2)
	// 0.19947114020071635
	// >>> gaussian(1, 2)
	// 0.17603266338214976
	// >>> gaussian(2, 2)
	// 0.12098536225957168
	
	// Parameters rounded to 20, 18, 12, which in turn are relatively the same at 10, 9, and 6.
	
	// 5x5 smoothing kernel, as suggested in "ORB: an efficient alternative to SIFT or SURF"
	public final static int[] SMOOTH_KERNEL = new int[]{6, 9, 10, 9, 6};
	
	default public T gaussianSmoothed() {
		return twoPassConvolve(SMOOTH_KERNEL, SMOOTH_KERNEL);
	}
}
