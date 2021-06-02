/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package tempixTerminalPackage;

import java.io.*;
import com.fazecast.jSerialComm.*;
/**
 *
 * @author peetz
 */
public class JSerialCommsController  {
    
    private static SerialPort comPort = null;
    private int rcvMsgAmt = 0;
    
    public JSerialCommsController() {
        super();
    }
    
    // TODO  next method needs tobe successfully called somehow before usage of comPort elsewhere    
    // before debugging can start !  ie. open must be called 
    
    public   String listPortsNOpenFirstNOnly()   
    {
        String res = "";
        SerialPort[] ports;
        SerialPort sPort;
        try {
            ports = SerialPort.getCommPorts();
            for (int i1 = 0; i1 < ports.length; i1++) {
                sPort = ports[i1];
                if ((comPort == null                 ) && (sPort.getSystemPortName().startsWith("ttyUSB"))) {
                    comPort = sPort;
                }
                res=res.concat(sPort.getSystemPortName()  +  " - " +  sPort.getPortDescription()+"\n" );           
            }
           res = res.concat("amount found ports: "+ports.length+"\n");
           
           if ((comPort != null) && (!comPort.isOpen()) ) {
                comPort = SerialPort.getCommPorts()[0];  // assumed to be the only one, ok for debuggging tempix
                                                          //   else let the user choose on UI which to open
                res = res.concat("trying port "+comPort.getSystemPortName() + "\n");
                comPort.setComPortParameters(9600, 8, comPort.ONE_STOP_BIT, comPort.NO_PARITY);  
                comPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0x00FFFFFF ,0 );  
                // timeouts tobe tested since documentation are very rare missing messageTimeout and intercharacterTimeout
                // so lets see what comes
                if (comPort.openPort()) {
                    comPort.addDataListener(new SerialPortDataListenerWithExceptions() {
                        @Override
                        public int getListeningEvents() { return SerialPort.LISTENING_EVENT_DATA_AVAILABLE; }
                        @Override
                        public void serialEvent(SerialPortEvent event)
                        {
                           if (event.getEventType() == SerialPort.LISTENING_EVENT_DATA_AVAILABLE)  {
                               // assumed to run over EDT for threadsafety reasons  ? TODO debugged and tested
                               
                               try {
                                   String res2;
                                    ++ rcvMsgAmt;
                                     byte[] newData = new byte[comPort.bytesAvailable()];
                                     int numRead = comPort.readBytes(newData, newData.length);
                                     if (numRead > 0) {
                                        res2 = new String(newData);
                                    //    res2 = "rcv msg nr "+rcvMsgAmt+ ": "+res2 + "\n";  // for debugging purpose only
                                        TempixTerminal.availableDataReceived(res2);
                                     }
                               } catch (Throwable ex)  {
                                    String res3;
                                    res3 = "exception during serialEvent: " + ex.getMessage() +"\n";
                                    TempixTerminal.addToLogger(res3);
                               }    
                               
                           }  
                        }
                        @Override
                        public void catchException​(Exception ex) {
                            TempixTerminal.addToLogger("exception in SerialPort..Listener..catchException: " 
                                                            + ex.getMessage());
                            StringWriter sw = new StringWriter();
                            PrintWriter pw = new PrintWriter(sw);
                            ex.printStackTrace(pw);
                            String sStackTrace = sw.toString(); 
                            TempixTerminal.addToLogger(sStackTrace +"\n");
                            JSerialCommsController.closePort();
                        }
                    });
                    res = res.concat("sucessfully opened: "+SerialPort.getCommPorts()[0] +"\n");
                } else {
                    comPort = null;
                }
                if (comPort == null)  {
                    res = res.concat("comPort is null or unavailable\n");
                } else {
                    res= res.concat(comPort.getSystemPortName()+ " open state = "+comPort.isOpen()+"\n");
                }
           } else {
               if (comPort != null) {
                   res = res.concat("port "+ SerialPort.getCommPorts()[0] + " already open\n"  );
               } else {
                  res = res.concat("no com ports found\n"); 
               }
           }        
        }  catch (Throwable ex) { 
            res = res.concat("exception::"+ex.getMessage()+"\n");
        } 
        return res;    
    }
    
    public static String closePort()
    {
        String res = "";
        try {
            if (comPort != null) {
                if(comPort.isOpen()) {
                    if (comPort.closePort()) {
                        res = res.concat("port "+ comPort.getSystemPortName()+ " closed\n");
                    } else {
                       res = res.concat("port "+ comPort.getSystemPortName()+ " could not be closed\n");
                    }
                }  else {
                    res = res.concat("port "+ comPort.getSystemPortName()+ " is not open\n");
                }
                comPort = null;
            }   else {
                res = res.concat("port was null\n");
            }
        } catch (Throwable ex){
            res = res.concat("exception during closePort: "+ex.getMessage()+"\n");
        }
    
        return res;
    }
            
    public static void sendString(String str)
    {
        if (comPort.isOpen()) {
            byte[] bytes = str.getBytes();  
            comPort.writeBytes(bytes, bytes.length);
        }  else {
            TempixTerminal.addToLogger("comPort is not open \n");
        }
    }
    
    public  static void pingTest(String st) 
    {
        sendString("@ping: "+st);
    }
   
    
}
