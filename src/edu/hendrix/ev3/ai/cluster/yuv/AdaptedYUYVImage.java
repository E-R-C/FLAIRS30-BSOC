package edu.hendrix.ev3.ai.cluster.yuv;

import java.util.function.IntBinaryOperator;
import java.util.function.IntUnaryOperator;

import edu.hendrix.ev3.ai.cluster.Clusterable;
import edu.hendrix.ev3.imgproc.ProcessableImage;
import edu.hendrix.ev3.util.DeepCopyable;
import edu.hendrix.ev3.util.Util;
import javafx.scene.paint.Color;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.video.YUYVImage;

public class AdaptedYUYVImage extends YUYVImage implements ProcessableImage<AdaptedYUYVImage>, DeepCopyable<AdaptedYUYVImage>, Clusterable<AdaptedYUYVImage> {
	private byte[] pix;
	
	public static byte[] pixelCopy(byte[] pixels) {
		byte[] newPix = new byte[pixels.length];
		for (int i = 0; i < pixels.length; i++) {
			newPix[i] = pixels[i];
		}
		return newPix;
	}
	
	public static int numPixels(int width, int height) {
		return width * height * 2;
	}
	
	public AdaptedYUYVImage(int width, int height) {
		this(new byte[numPixels(width, height)], width, height);
	}
	
	public AdaptedYUYVImage(byte[] pix, int width, int height) {
		super(pix, width, height);
		this.pix = pix;
	}
	
	public AdaptedYUYVImage(AdaptedYUYVImage other) {
		this(pixelCopy(other.copyBytes()), other.getWidth(), other.getHeight());
	}
	
	public AdaptedYUYVImage(YUYVPixelMaker creator, int width, int height) {
		this(width, height);
		for (int i = 0; i < pix.length; i++) {
			int x = (i % (width*2)) / 2;
			int y = i / (width*2);
			YUV what = i % 2 == 0 ? YUV.Y : i % 4 == 1 ? YUV.U : YUV.V;
			pix[i] = creator.apply(x, y, what);
		}
	}
	
	@Override
	public String toString() {
		StringBuilder result = new StringBuilder();
		result.append(getWidth());
		result.append(" ");
		result.append(getHeight());
		for (int i = 0; i < pix.length; i++) {
			result.append(' ');
			result.append(pix[i]);
		}
		return result.toString();
	}
	
	public static AdaptedYUYVImage fromString(String src) {
		String[] nums = src.split(" ");
		int width = Integer.parseInt(nums[0]);
		int height = Integer.parseInt(nums[1]);
		int remainingValues = nums.length - 2;
		if (numPixels(width, height) > remainingValues) {
			throw new IllegalArgumentException(String.format("Badly formatted AdaptedYUYVImage: w:%d h:%d p:%d", width, height, remainingValues));
		}
		
		byte[] pix = new byte[numPixels(width, height)];
		for (int i = 0; i < pix.length; i++) {
			pix[i] = Byte.parseByte(nums[i+2]);
		}

		AdaptedYUYVImage result = new AdaptedYUYVImage(pix, width, height);
		return result;
	}
	
	@Override
	public void setRGB(int x, int y, Color c) {
		//double Y = 0.299 * c.getRed() + 0.587 * c.getGreen() + 0.114 * c.getBlue();
		//double U = Math.max(0, 0.492 * (c.getBlue() - Y));
		//double V = Math.max(0, 0.877 * (c.getRed() - Y));
		double Y = (c.getRed() + c.getBlue() + c.getGreen()) / 3;
		double U = (1.0 + c.getBlue() - Y) / 2;
		double V = (1.0 + c.getRed() - Y) / 2;
		int base = getPairBase(x, y);
		pix[base] = (byte)(Y * 255);
		pix[base+1] = (byte)(U * 255);
		if (base + 3 < pix.length) {
			pix[base+2] = pix[base];
			pix[base+3] = (byte)(V * 255);
		}
	}

	public int getPairBase(int x, int y) {  
        return 2 * (y * getWidth() + (x - x % 2));  
    }  
	
	public void display(GraphicsLCD display) {
		super.display(display, 0, 0, super.getMeanY());
	}
	
	public byte[] copyBytes() {
		byte[] result = new byte[pix.length];
		for (int i = 0; i < pix.length; i++) {
			result[i] = pix[i];
		}
		return result;
	}
	
	int getColumn(int pixel) {
		return (pixel / 2) % getWidth();
	}
	
	int getRow(int pixel) {
		return (pixel / 2) / getWidth();
	}
	
	@Override
	public AdaptedYUYVImage shrunken(int shrinkFactor) {
		Util.assertArgument(canShrinkBy(shrinkFactor), "Uneven shrinkage: " + shrinkFactor + " (" + getWidth() + "," + getHeight() + ")");
		 
		AdaptedYUYVImage shrunk = new AdaptedYUYVImage(getWidth() / shrinkFactor, getHeight() / shrinkFactor);
		int p = 0;
		for (int i = 0; i < pix.length; i+=4) {
			if (getRow(i) % shrinkFactor == 0 && getColumn(i/4) % shrinkFactor == 0) {
				for (int j = 0; j < 4; j++) {
					shrunk.pix[p++] = pix[i + j];
				}
			}
		}
		return shrunk;
	}
	
	public AdaptedYUYVImage xConvolve1D(final int[] kernel) {
		return convolve1D(kernel, i -> 2 * (i - kernel.length/2));
	}
	
	public AdaptedYUYVImage yConvolve1D(final int[] kernel) {
		return convolve1D(kernel, i -> 2 * getWidth() * (i - kernel.length/2));		
	}
	
	public AdaptedYUYVImage convolve1D(final int[] kernel, IntUnaryOperator srcIndex) {
		byte[] convolved = new byte[pix.length];
		for (int i = 0; i < pix.length; i++) {
			if (i % 2 == 0) {
				convolved[i] = convolvePixel(i, kernel, srcIndex);
			} else {
				convolved[i] = pix[i];
			}
		}
		return new AdaptedYUYVImage(convolved, getWidth(), getHeight());
	}
	
	private byte convolvePixel(int index, final int[] kernel, IntUnaryOperator srcIndex) {
		int numerator = 0, denominator = 0;
		for (int i = 0; i < kernel.length; i++) {
			int src = index + srcIndex.applyAsInt(i);
			if (src >= 0 && src < pix.length) {
				numerator += kernel[i] * Byte.toUnsignedInt(pix[src]);
				denominator += kernel[i];
			}
		}
		return (byte)(denominator == 0 ? pix[index] : numerator / denominator);
	}
	
	@Override
	public int getY(int x, int y) {
		return super.getY(x, y) & 0xFF;
	}
	
	@Override
	public int getIntensity(int x, int y) {
		return getY(x, y);
	}
	
	@Override
	public int getU(int x, int y) {
		return super.getU(x, y) & 0xFF;
	}
	
	@Override
	public int getV(int x, int y) {
		return super.getV(x, y) & 0xFF;
	}
	
	public static int clamp(int value) {
		return Math.min(255, Math.max(0, value));
	}
	
	public Color getRGBColor(int x, int y) {
		int c = getY(x, y) - 16;
		int d = getU(x, y) - 128;
		int e = getV(x, y) - 128;
		int r = clamp((298*c + 409*e + 128) >> 8);
		int g = clamp((298*c - 100*d - 208*e + 128) >> 8);
		int b = clamp((298*c + 516*d + 128) >> 8);
		return new Color(r / 255.0, g / 255.0, b / 255.0, 1.0);
	}

	@Override
	public AdaptedYUYVImage weightedCentroidWith(AdaptedYUYVImage other, long thisCount, long otherCount) {
		AdaptedYUYVImage combo = combine(this, other, (p1, p2) -> {
			long num = p1 * thisCount + p2 * otherCount;
			long den = thisCount + otherCount;
			long quo = num / den;
			long mod = num % den;
			if (mod > den/2) {quo += 1;}
			return (byte)quo;
		});
		return combo;
	}
	
	public static void checkCompatibleImages(AdaptedYUYVImage img1, AdaptedYUYVImage img2) {
		if (img1.getWidth() != img2.getWidth() || img1.getHeight() != img2.getHeight()) {
			throw new IllegalArgumentException("Images of unequal dimensions");
		}
	}
	
	// Helper functions
	public static AdaptedYUYVImage combine(AdaptedYUYVImage img1, AdaptedYUYVImage img2, IntBinaryOperator combiner) {
		checkCompatibleImages(img1, img2);
		byte[] newPix = new byte[img2.copyBytes().length];
		for (int i = 0; i < newPix.length; i++) {
			int combo = combiner.applyAsInt(Byte.toUnsignedInt(img1.pix[i]), Byte.toUnsignedInt(img2.pix[i]));
			newPix[i] = (byte)combo;
		}
		return new AdaptedYUYVImage(newPix, img1.getWidth(), img1.getHeight());
	}

	@Override
	public AdaptedYUYVImage deepCopy() {
		return new AdaptedYUYVImage(this);
	}
}
