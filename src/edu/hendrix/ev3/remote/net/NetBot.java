package edu.hendrix.ev3.remote.net;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;

import edu.hendrix.ev3.remote.Move;
import edu.hendrix.ev3.remote.Mover;
import edu.hendrix.ev3.util.Logger;
import edu.hendrix.ev3.util.Util;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

abstract public class NetBot {
	private DatagramChannel channel;
	private ByteBuffer inputBuffer;
	private int inputSize, outputSize;
	private boolean quit, livefeed = false;
	private Move currentMove;
	private InetAddress sender;
	private int cycles;
	private long startTime;
	
	
	public NetBot(int incomingSize, int outgoingSize) throws IOException {
		channel = DatagramChannel.open();
		channel.socket().bind(new InetSocketAddress(RobotConstants.MSG_PORT));
		inputBuffer = ByteBuffer.allocate(incomingSize);
		inputSize = incomingSize;
		outputSize = outgoingSize;
		cycles = 0;
	}
	
	public void resetTimer() {
		startTime = System.currentTimeMillis();
		cycles = 0;
		Logger.EV3Log.markTime();
	}
	
	/**
	 * Called before the main loop begins.
	 */
	abstract public void setup();
	
	/**
	 * Called after the main loop ends.
	 */
	abstract public void teardown();
	
	/**
	 * Check sensors and update state accordingly for determining appropriate
	 * action selection in selectMove().
	 */
	abstract public void checkSensors();
	
	/** 
	 * 
	 * @param receivedMessage
	 * receivedMessage will be of length zero if no bytes were received
	 * @return the robot's instructions
	 */
	abstract public NetBotCommand selectMoveAndReply(byte[] receivedMessage);
	
	public int getCycle() {return cycles;}
	
	public Move getCurrentMove() {return currentMove;}
	
	public void print(String msg) {
		print(msg, 4);
	}
	
	public void print(String msg, int line) {
		LCD.drawString(msg + "        ", 0, line);
	}
	
	public void setLivefeed(boolean b){
		this.livefeed = b;
	}
	public void mainLoop() {
		print("Setting up");
		setup();
		print("Ready!");
		resetTimer();
		try {
			while (!quit && !Button.ESCAPE.isDown()) {
				cycles += 1;
				checkSensors();
				byte[] incoming = resolveMessages();
				NetBotCommand response = selectMoveAndReply(incoming);
				currentMove = response.getMove();
				print(currentMove.name(),5);
				Mover.move(currentMove);
				if (response.hasMessage()) {
					transmitReply(response.getMessage());
				}
				if(livefeed){
					sendCurrentMove();
				}
				quit = response.shouldQuit();
				
			}
			Mover.move(Move.STOP);
			logFPS();
			teardown();
		} catch (Exception exc) {
			Mover.move(Move.STOP);
			exc.printStackTrace();
		}
	}
	private void sendCurrentMove() throws IOException{
		byte[] b = new byte[2];
		b[0] = -2;
		b[0] = (byte) currentMove.ordinal();
		transmitReply(b);
	}
	protected byte[] resolveMessages() throws IOException {
		inputBuffer.clear();
		channel.configureBlocking(false);
		SocketAddress src = channel.receive(inputBuffer);
		if (src == null) {
			return new byte[0];
		} else {
			sender = ((InetSocketAddress)src).getAddress();
			// Util.assertState(inputBuffer.array().length == inputSize, "Size mismatch on input");
			return inputBuffer.array();
		}
	}
	protected boolean liveFeeding(){
		return livefeed;
	}
	protected void transmitReply(byte[] reply) throws IOException {
		if (sender != null) {
			// Util.assertArgument(reply.length == outputSize, "Size mismatch on output");
			DatagramPacket info = new DatagramPacket(reply, reply.length, sender, RobotConstants.MSG_PORT);
			channel.configureBlocking(true);
			channel.socket().send(info);
		}
	}
	
	public void logFPS() {
		long duration = System.currentTimeMillis() - startTime;
		double fps = 1000.0 * cycles / duration;
		Logger.EV3Log.format("%4.2f hz (%d cycles / %d ms)", fps, cycles, duration);		
	}
}
