package edu.hendrix.ev3.remote.net;

import java.io.IOException;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.function.Function;

public class ClientSender<U extends TaggedMessage, R extends TaggedMessage> implements UpdateListener<R> {
	private DatagramSocket sock;
	private UpdateThread<R> updater;
	private byte pendingTag, receivedTag;
	private R lastReply;
	
	public ClientSender(Function<byte[],R> converter, int expectedBytes) throws SocketException {
		sock = new DatagramSocket(RobotConstants.MSG_PORT);
		updater = new UpdateThread<>(sock, converter, expectedBytes);
		updater.addListener(this);
		updater.start();
		pendingTag = receivedTag = 0;
	}
	
	public void addUpdateListener(UpdateListener<R> listener) {
		updater.addListener(listener);
	}
	
	public void send(U msg) throws IOException {
		pendingTag = msg.getTag();
		receivedTag = (byte) (pendingTag - 1);
		msg.ship(sock);
	}
	
	public boolean waitingForReply() {return pendingTag != receivedTag;}
	
	public R getLastReply() {return lastReply;}
	
	@Override
	public void report(R msg) {
		receivedTag = msg.getTag();
		lastReply = msg;
	}
}
