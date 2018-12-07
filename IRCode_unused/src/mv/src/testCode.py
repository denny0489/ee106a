import serial #To install

def main():

	done = False

	while (done != True):
		try:
			arduino = serial.Serial('COM6', 9600, timeout = 1)
	        # arduino = serial.Serial("/dev/ttyACMO", 9600, timeout = 1)
		except:
			# print("Name = ", arduino.name)
			print("Error when getting data from arduino")
			continue;

		if (arduino.readLine().length() == 0):
			print ("No data from arduino")
			continue

		println("Data from arduino = " + arduino.readLine())
		done = True

if __name__ == "__main__":
	main()