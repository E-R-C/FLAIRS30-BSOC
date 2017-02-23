package edu.hendrix.ev3.ai.cluster.yuv;

public enum YUV {
	Y {
		@Override
		public int getValue(AdaptedYUYVImage img, int x, int y) {
			return img.getY(x, y);
		}
	}, U {
		@Override
		public int getValue(AdaptedYUYVImage img, int x, int y) {
			return img.getU(x, y);
		}
	}, V {
		@Override
		public int getValue(AdaptedYUYVImage img, int x, int y) {
			return img.getV(x, y);
		}
	};
	
	abstract public int getValue(AdaptedYUYVImage img, int x, int y);
}
