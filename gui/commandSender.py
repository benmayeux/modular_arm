import serial
import time


class commandSender():

    def __init__(self,comPort) -> None:
        self.arduino = serial.Serial(port='COM' + str(comPort), baudrate=9600, timeout=0.1)

    # write bytes
    def write_read(self,x):
        self.write(x)
        time.sleep(0.05)
        return self.read()

    def write(self, x):
        self.arduino.write(bytes(x,'utf-8'))

    def read(self):
        data = self.arduino.read_all()
        return data

    def close(self):
        self.arduino.close()

    # Monitor for bytes
    def monitor(self):
        if(self.arduino.in_waiting > 0):
            print(str(self.arduino.read_all(), 'utf-8'))



def main():
    cs = commandSender(12)

    while(True):
        command = input("Enter a command: ") # Taking input from user
        value = cs.write_read(command)
        print(str(value, 'utf-8')) # printing the value

        if(command == "end"):
            break

    cs.close()


if(__name__ == "__main__"):
    main()