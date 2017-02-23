package edu.hendrix.ev3.ai.cluster;

import java.util.function.IntBinaryOperator;

import edu.hendrix.ev3.ai.cluster.yuv.AdaptedYUYVImage;
import edu.hendrix.ev3.ai.cluster.yuv.YUV;

public class EdgeImage {
	private int[][] edges;
	private int width, height;
	
	public EdgeImage(int width, int height) {
		this.width = width;
		this.height = height;
		edges = new int[width][height];
	}
	
	public EdgeImage(EdgeImage other) {
		this(other.width, other.height);
		forEach((x, y) -> edges[x][y] = other.edges[x][y]);
	}
	
	public EdgeImage(AdaptedYUYVImage src) {
		this(src.getWidth(), src.getHeight());
		for (int x = 1; x < width - 1; x++) {
			for (int y = 1; y < height - 1; y++) {
				edges[x][y] = gradientAt(src, x, y);
			}
		}
	}
	
	public AdaptedYUYVImage toYUYV() {
		return new AdaptedYUYVImage((int x, int y, YUV yuv) -> {
			return yuv == YUV.Y ? byteValue(x, y) : Byte.MAX_VALUE;
		}, width, height);
	}
	
	public byte byteValue(int x, int y) {
		return (byte)(Math.min(0xFF, get(x, y)) & 0xFF);
	}
	
	public int get(int x, int y) {return edges[x][y];}
	public int getWidth() {return width;}
	public int getHeight() {return height;}
	
	public int getMaxValue() {
		return fold((x, y, max) -> Math.max(max, get(x, y)), Integer.MIN_VALUE);
	}
	
	public int getMinValue() {
		return fold((x, y, min) -> Math.min(min, get(x, y)), Integer.MAX_VALUE);
	}
	
	public int getValueRange() {
		return getMaxValue() - getMinValue();
	}
	
	int gradientAt(AdaptedYUYVImage src, int x, int y) {
		int upSum = -src.getY(x-1, y-1) - 2*src.getY(x, y-1) - src.getY(x+1, y-1);
		int downSum = src.getY(x-1, y+1) + 2*src.getY(x, y+1) + src.getY(x+1, y+1);
		int ySum = Math.abs(upSum + downSum);

		int leftSum = -src.getY(x-1, y-1) - 2*src.getY(x-1, y) - src.getY(x-1, y+1);
		int rightSum = src.getY(x+1, y-1) + 2*src.getY(x+1, y) + src.getY(x+1, y+1);
		int xSum = Math.abs(leftSum + rightSum);
		
		return ySum + xSum;
	}
	
	public void forEach(IntBinaryOperator pixOp) {
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				edges[x][y] = pixOp.applyAsInt(x, y);
			}
		}
	}
	
	public <T> T fold(BinaryFoldingOperator<T> op, T start) {
		T result = start;
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				result = op.apply(x, y, result);
			}
		}
		return result;
	}
}
