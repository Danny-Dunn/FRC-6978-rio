package frc;

import java.io.DataInputStream;

//import java.net;
import java.net.Socket;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//link protocol 0.1 compliant
public class VisionInterface implements Runnable {
    
    class LocationProperty {
        public double x;
        public double y;
        public double z;
    }
    
    class SizeProperty {
        public double radius;
    }
    
    class ObjectCreateReport {
        public LocationProperty location;
        public SizeProperty size;
    }
    
    Socket sock;
    DataInputStream diStream;

    enum ReceiveState {
        offline,
        running,
        inframe,
        failed
    };

    ReceiveState currentState = ReceiveState.offline; 

    void initComms() {
        try {
            sock = new Socket("vcomp.local", 5801);
            diStream = new DataInputStream(sock.getInputStream());
            currentState = ReceiveState.running;
            System.out.println("[VInterface] VC comm up");
        } catch(Exception e) {
            System.out.println("[VInterface] VC link init failed!");
            currentState = ReceiveState.failed;
        }
    }

    int frameID;
    int reportID;
    int reportType;
    int propCount;
    int propType;


    public void run() {
        while(true) {
            switch(currentState) {
                case failed:
                    break;
                case offline:
                    initComms();
                    break;
                case running:
                    try{
                        frameID = diStream.readInt();
                        reportID = diStream.readInt();
                        reportType = diStream.readByte();
                        propCount = diStream.readByte();
                        switch(reportType) {
                            case 2: //object_create
                                ObjectCreateReport report = new ObjectCreateReport();
                                
                                report.location = new LocationProperty();
                                report.location.x = diStream.readDouble();
                                report.location.y = diStream.readDouble();
                                report.location.z = diStream.readDouble();

                                report.size = new SizeProperty();
                                report.size.radius = diStream.readDouble();
                                SmartDashboard.putNumber("Report radius", report.size.radius);
                                break;
                        }
                        
                    } catch(Exception e) {
                        
                    }
                    break;
                default:
                    break;
            }
        }
    }
}
