package edu.hendrix.ev3.remote.net.actionselector;

import java.nio.ByteBuffer;
import org.joda.time.LocalDateTime;

import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.remote.net.ReturnableTaggedMessage;
import edu.hendrix.ev3.util.StampedStorage;
import edu.hendrix.ev3.util.Util;

public class ActionSelectorCommand extends ReturnableTaggedMessage {
	
	public final static int SIZE = 3 + StampedStorage.DATE_TIME_BYTES + 3 * Integer.BYTES;

	private Mode mode;
	private Move move;
	private LocalDateTime stamp;
	private int suffix;
	private byte tag;
	private int numClusters, shrinkFactor;
	
	public ActionSelectorCommand(Mode mode, Move move, LocalDateTime stamp, int suffix, byte tag, int numClusters, int shrinkFactor) {
		this.mode = mode;
		this.move = move;
		this.stamp = stamp;
		this.suffix = suffix;
		this.tag = tag;
		this.numClusters = numClusters;
		this.shrinkFactor = shrinkFactor;
	}
	
	public ActionSelectorCommand(byte[] bytes) {
		Util.assertArgument(bytes.length == SIZE, "Message size: " + bytes.length + " should be " + SIZE);
		ByteBuffer buffer = ByteBuffer.wrap(bytes);
		stamp = StampedStorage.getFrom(buffer);
		suffix = buffer.getInt();
		tag = buffer.get();
		move = Move.values()[buffer.get()];
		mode = Mode.values()[buffer.get()];
		numClusters = buffer.getInt();
		shrinkFactor = buffer.getInt();
	}

	@Override
	public byte[] toBytes() {
		ByteBuffer bytes = ByteBuffer.allocate(SIZE);
		StampedStorage.putInto(stamp, bytes);
		bytes.putInt(suffix);
		bytes.put(tag);
		bytes.put((byte)move.ordinal());
		bytes.put((byte)mode.ordinal());
		bytes.putInt(numClusters);
		bytes.putInt(shrinkFactor);
		return bytes.array();
	}

	@Override
	public byte getTag() {
		return tag;
	}

	@Override
	public boolean keepGoing() {
		return mode != Mode.QUIT;
	}

	public LocalDateTime getStamp() {return stamp;}
	public int getSuffix() {return suffix;}
	public Move getMove() {return move;}
	public Mode getMode() {return mode;}
	public int getNumClusters() {return numClusters;}
	public int getShrinkFactor() {return shrinkFactor;}
}
