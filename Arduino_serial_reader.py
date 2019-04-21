import serial


def main():

    # declare port, baud, and file path
    serial_port = '/dev/cu.usbmodem14201'
    baud_rate = 9600
    write_to_file_path = "Samsung_18650_set3_run_1.csv"

    # open file for writing and  get serial feed
    output_file = open(write_to_file_path, "w+")
    ser = serial.Serial(serial_port, baud_rate)

    # write serial output to a csv file
    output_file.write("Data Point" + "," + "Voltage" + "\n")
    while True:
        line = ser.readline()
        line = line.decode("utf-8")
        data_line = line
        print(line, end="")
        output_file.write(line)
        voltage = float(data_line.split(",")[1])
        # when batteries have been depleted
        if voltage < 2.40:
            break
    output_file.close()

main()


