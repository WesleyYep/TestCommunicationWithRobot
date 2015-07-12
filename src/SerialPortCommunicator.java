import jssc.SerialPort;
import jssc.SerialPortException;

public class SerialPortCommunicator {

	private SerialPort currentSerialPort = null;

	public boolean openPort(String portName) {
		currentSerialPort = new SerialPort(portName);
		try {	
			if (currentSerialPort.openPort()) {
				//setup the port setting
				currentSerialPort.setParams(
						SerialPort.BAUDRATE_115200,
						SerialPort.DATABITS_8,
						SerialPort.STOPBITS_1,
						SerialPort.PARITY_NONE
						);
				return true;
			} else {
				return false;
			}
		} catch (SerialPortException ex) {
			System.out.println(ex);
			return false;
		}
	}


	public boolean writeData(int[] data) {
		if (currentSerialPort != null && currentSerialPort.isOpened() == true) {
			//System.out.println(System.currentTimeMillis());
			try {
				currentSerialPort.writeIntArray(data);
                //System.out.println(System.currentTimeMillis());
				return true;
			} catch (SerialPortException e) {
				e.printStackTrace();
				return false;
			}
		} else {
			return false;
		}
	}

	public boolean closePort() {
		try {
			if (currentSerialPort != null && currentSerialPort.closePort()) {
				return true;
			} else {
				return false;
			}
		} catch (SerialPortException e1) {
			e1.printStackTrace();
			return false;
		}
	}

}
