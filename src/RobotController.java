import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class RobotController {
	
	private SerialPortCommunicator communicator;
    private static boolean isSending = false;

    public static void main (String[] args) {
        RobotController rc = new RobotController(new SerialPortCommunicator());
        double[] leftWheelVelocity = new double[] {1,1,1,1,1,0,0,0,0,0,0};
        double[] rightWheelVelocity = new double[] {1,1,1,1,1,0,0,0,0,0,0};
        double[] linearVelocity = new double[] {1,1,1,1,1,0,0,0,0,0,0};
        double[] angularVelocity = new double[] {0,0,0,0,0,0,0,0,0,0,0};
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));

        while (true) {
            if (isSending) {
         //       rc.sendVelocity(leftWheelVelocity, rightWheelVelocity);
                rc.sendVelocity(linearVelocity, angularVelocity);
            } else {
                try {
                    String line = br.readLine();
                    if (line.startsWith("COM")) {
                        isSending = true;
                        rc.communicator.openPort(line);
                        System.out.println("Opening port " + line);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

	public RobotController(SerialPortCommunicator c) {
		communicator = c;
	}

  //  public void sendVelocity(double[] leftWheelVelocity, double[] rightWheelVelocity) {
        public void sendVelocity(double[] linearVelocity, double[] angularVelocity) {

		int[] sdata = new int[48];
		
		sdata[0] = 255;
		sdata[1] = 255;
		sdata[2] = 0x01;
		
		
		int checksum = (sdata[0] +sdata[1] + sdata[2]) & 0xff;
		
		for (int i=0; i < 11; i++) {
//            double tempvr = (int)(leftWheelVelocity[i]*126);
//            double tempvl = (int)(rightWheelVelocity[i]*126);
//            //tempvr = 0;
//            //tempvl = 0;
//
//            if( tempvr > 511 ) tempvr = 511;
//            if( tempvr <-511 ) tempvr =-511;
//            if( tempvl > 511 ) tempvl = 511;
//            if( tempvl <-511 ) tempvl =-511;
//
//            sdata[3*i  +3] =  (((int)(tempvr) & 0xff));//(BYTE)(tempvr & 0xff);
//            sdata[3*i+1+3] = (((int)(tempvr) & 0xff));
//            sdata[3*i+2+3] = (int)(tempvr*0x0300)>>>8 + (int)(tempvl*0x0300)>>>6; //(BYTE)((tempvr & 0x0300) >> 8) + (BYTE)((tempvl & 0x0300) >> 6);

			double tempLin = linearVelocity[i];
			double tempAng = angularVelocity[i];

			if (tempLin < -4.0) {
				tempLin = -4.0;
			} else if (tempLin > 4.0) {
				tempLin =  4.0;
			}

			// Clip the angle. Max 128, min -128.
			if (tempAng < -128.0) {
				tempAng = -128.0;
			} else if (tempAng > 128.0) {
				tempAng =  128.0;
			}

			//http://www.javamex.com/java_equivalents/unsigned.shtml
			sdata[4*i+0+3] = (((int)(tempLin*2048.)) & 0xff);
			sdata[4*i+1+3] = (((int)(tempLin*2048.)>>>8) & 0xff);
			sdata[4*i+2+3] = (((int)(tempAng*64.)) & 0xff);
			sdata[4*i+3+3] = (((int)(tempAng*64.)>>>8) & 0xff);

			checksum += sdata[4*i+3];
			checksum += sdata[4*i+1+3];
			checksum += sdata[4*i+2+3];
			checksum += sdata[4*i+3+3];
		}
		
		sdata[47] = -checksum & 0xff;

//        sdata[33+3] =	(27 & 0xff);
//        sdata[34+3] =	(2 & 0xff);
//        sdata[35+3] =	(8 & 0xff);
		
		communicator.writeData(sdata);
	}

}
