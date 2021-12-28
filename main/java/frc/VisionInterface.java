package frc;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStream;
import java.io.InputStreamReader;
//import java.net;
import java.net.Socket;
import java.nio.Buffer;
import java.nio.ByteBuffer;

public class VisionInterface implements Runnable {
    Socket sock;
    InputStream piIn;

    ByteBuffer reportBuf;

    int currentPacketLength;
    int currentCaputuredLength;

    enum ReceiveState {
        offline,
        await,
        inframe,
        failed
    };

    ReceiveState currentState = ReceiveState.offline; 

    void initComms() {
        try {
            sock = new Socket("vcomp.local", 5660);
            piIn = sock.getInputStream();
            currentState = ReceiveState.await;
        } catch(Exception e) {
            System.out.println("[VInterface] RPi comm init failed!");
            currentState = ReceiveState.failed;
        }
    }

    void catchPacket() {
        try {
            currentCaputuredLength = reportBuf.position() + 1;
            if (piIn.available() > 0) {
                reportBuf.put(piIn.readAllBytes());
            }
        } catch (Exception e) {
            System.out.println("[VInterface] Error reading while packet in-flight shutting down");
            currentState = ReceiveState.failed;
        }

    }

    public void run() {
        while(true) {
            switch(currentState) {
                case failed:
                    break;
                case offline:
                    initComms();
                    break;
                case await:
                    try{
                        if(piIn.available() > 2) { //begin packet
                            reportBuf.position(0);
                            reportBuf.put(piIn.readAllBytes());
                            currentCaputuredLength = reportBuf.position() + 1;
                            reportBuf.position(0);
                            if(reportBuf.get() == 0x5b) {
                                currentPacketLength = reportBuf.get();
                                currentState = ReceiveState.inframe;
                            } else {
                                System.out.println("[VInterface] Preamble incorrect");
                            }
                        }
                    } catch(Exception e) {
                        System.out.println("[VInterface] Transaction failed or malformed");
                        reportBuf.position(0);
                        currentState = ReceiveState.await;
                    }
                    break;
                case inframe:
                    catchPacket();
                    break;
            }
        }
    }
}
