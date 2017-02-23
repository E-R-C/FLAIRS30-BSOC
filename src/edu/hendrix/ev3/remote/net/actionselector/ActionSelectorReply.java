package edu.hendrix.ev3.remote.net.actionselector;

import java.nio.ByteBuffer;
import org.joda.time.LocalDateTime;
import java.util.ArrayList;

import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.remote.net.TaggedMessage;
import edu.hendrix.ev3.util.Duple;
import edu.hendrix.ev3.util.FixedSizeArray;
import edu.hendrix.ev3.util.StampedStorage;

public class ActionSelectorReply implements TaggedMessage {
	
	public final static int MAX_STORED = 30;
	public final static int SIZE = 1 + MAX_STORED * (StampedStorage.DATE_TIME_BYTES + Integer.BYTES);
	
	private FixedSizeArray<Duple<LocalDateTime,Integer>> names;
	private byte tag;
	private boolean isPulse;
	private LocalDateTime pulseTime;
	private Move pulseMove;
	
	public ActionSelectorReply(byte tag) {
		this.tag = tag;
		names = FixedSizeArray.makeImmutableType(MAX_STORED);
	}
	public ActionSelectorReply(){
		isPulse = true;
	}
	public void addName(LocalDateTime localDateTime, int suffix) {
		names.add(new Duple<LocalDateTime,Integer>(localDateTime, suffix));
	}
	
	public void makePulse(){
		isPulse = true;
	}
	public void setPulseMove(Move m){
		pulseMove = m;
	}
	public ArrayList<Duple<LocalDateTime,Integer>> getNames() {
		return names.values();
	}
	public void setPulseTime(LocalDateTime pulseTime){
        this.pulseTime = pulseTime;
    }
	@Override
	public byte getTag() {
		return tag;
	}

	@Override
	public byte[] toBytes() {
		ByteBuffer bytes = ByteBuffer.allocate(SIZE);
		if (isPulse){
			bytes.put((byte)-1);
			LocalDateTime msg = LocalDateTime.now();
			StampedStorage.putInto(msg, bytes);
		} else {
			bytes.put(tag);
	        bytes.putInt(names.size());
			for (Duple<LocalDateTime,Integer> name: names.values()) {
				StampedStorage.putInto(name.getFirst(), bytes);
				bytes.putInt(name.getSecond());
			}
		}
		
		return bytes.array();
	}

	public static ActionSelectorReply fromBytes(byte[] bytes) {
		ActionSelectorReply result;
        ByteBuffer buffer = ByteBuffer.wrap(bytes);
        byte tag = buffer.get();
        if (tag == -1){
            result = new ActionSelectorReply();
            result.setPulseTime(StampedStorage.getFrom(buffer));
            Move moveFromPulse = Move.values()[buffer.get()];
            result.setPulseMove(moveFromPulse);
        } else {
            result = new ActionSelectorReply(tag);
            int numStored = buffer.getInt();
            for (int i = 0; i < numStored; i++) {
                result.addName(StampedStorage.getFrom(buffer), buffer.getInt());
            }
        }
        return result;
	}
}
