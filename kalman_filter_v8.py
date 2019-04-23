# DESCRIPTION
# -----------
# This program uses a Kalman Filter to estimate the state of charge (SoC) of two Li-ion batteries in series.
#
# OCV Measurement Frequency: 600s
# I Measurement Frequency: 10s
# Estimate Variance: [[5.0, 0], [0, 5.0]]
# Observation Variance: [[0.2, 0], [0, 0.2]
#
# CONTACT
# -------
# For inquiries, contact jogrady@usc.edu
#
# Jack O'Grady
# © 2019

# IMPORTS
# -------
import numpy as np
import serial
import time


# FUNCTIONS
# ---------

# Reads data from Serial (from Arduino)
# Inputs: the serial connection
# Returns: the decoded serial data
def readSerialLine(ser):
    line = ser.readline()
    line = line.decode("utf-8")
    dataLine = line
    lineOutput = dataLine.strip()
    return lineOutput


# Used to get the state of charge (SoC) from an open circuit voltage (OCV) reading,
# using a SoC(OCV) function calculated with an 8th order polynomial fit of the cell's quasi-OCV discharge at ~1/20C
# Inputs: the measured cell OCV from Arduino
# Returns: the state of charge SoC
def getSoCFromOCV(OCV):
    coefficients = [268.4970355259198, -6879.270367122276, 76716.49575172913, -486365.6733759814, 1917287.1707991716,
                    -4812471.06572991, 7511312.300797121, -6665390.783393391, 2574719.229612701]
    OCV_used = OCV
    # ensures that only OCVs within the bounds of the polynomial fit are used
    if OCV_used < 2.5:
        OCV_used = 2.5093
    if OCV_used > 4.057:
        OCV_used = 4.057
    # calculates the state of charge from the OCV
    SoC = 0
    i = 0
    while 8 - i >= 0:
        SoC += coefficients[i] * OCV_used ** (8 - i)
        i += 1
    # returns the state of charge as a function of measured open circuit voltage (OCV)
    return SoC


# FUNCTIONS FOR KALMAN FILTER OPERATIONS
# --------------------------------------

# Initializes the state matrix for the Kalman Filter
# Inputs: the initial measured state of charge SoC and current I
# Returns: the state matrix to be used in the Kalman Filter
def initStateVariable(SoC, I):
    return np.array([SoC], [I])


# Updates the Kalman Filter's transformation matrix with the appropriate time interval
# Inputs: the measured time interval between operations
# Returns: the correct transformation matrix to be used in Kalman Filter operations
def updateTransformationMatrix(deltaT):
    totalCoulombs = 10800
    return np.array([[1, -1 * (deltaT / totalCoulombs) * 100], [0, 1]])


# OVERVIEW OF MAIN
# ----------------
# 1. Initialize the Serial Port to communicate with the Arduino
# 2. Set up a CSV file for data recording
# 3. Get initial OCV and current measurement from Arduino sensors
# 4. Create the last state matrix and state estimate variance
# 5. Every 10s, measure the current and update the state estimate
# 6. Every 10min, measure the OCV and run an iteration of the Kalman Filter
# 7. Update the state estimate
# 8. Write the data to a CSV file

def main():

    # Initialize serial port
    serialPort = '/dev/cu.usbmodem14201'
    baudRate = 9600
    arduino = serial.Serial(serialPort, baudRate)
    time.sleep(2)

    # Set up CSV data output
    write_to_file_path = "Kalman_Filter_Comparison_5column_extra_time2.csv"
    output_file = open(write_to_file_path, "w+")
    estimate1Name = "Coulomb Counting"
    estimate2Name = "OCV"
    estimate3Name = "KF I=10s P=0.5 R=0.25"
    estimate4Name = "KF I=10s P=5.0 R=0.2"
    estimate5Name = "KF I=30s P=5.0 R=0.2"
    output_file.write("Time" + "," + estimate1Name + "," + estimate2Name + "," + estimate3Name + "," + estimate4Name +
                      "," + estimate5Name + "\n")

    # Set codes for the arduino output
    CURRENT_CODE = '1'
    OCV_CODE = '2'

    # INITIALIZE
    # ----------

    # Tell the Arduino to take the initial OCV measurement
    arduino.write(str.encode(OCV_CODE))
    while arduino.inWaiting() < 0:
        time.sleep(0.1)
    arduinoData = readSerialLine(arduino)
    initialVoltage = float(arduinoData) / 2 # divide by 2 since SoC(OCV) function is for 1 cell's OCV

    # calculate the initial SoC
    initialSoC = getSoCFromOCV(initialVoltage)

    # Tell the Arduino to take the initial current measurement
    arduino.write(str.encode(CURRENT_CODE))
    while arduino.inWaiting() < 0:
        time.sleep(0.1)
    arduinoData = readSerialLine(arduino)
    initialCurrent = float(arduinoData)

    # Display the initial SoC for the user
    displaySoC = round(initialSoC, 2)

    print("Battery Charge (" + estimate1Name + "): " + str(round(displaySoC, 2)) + "%")

    print("Battery Charge (" + estimate2Name + "): " + str(round(displaySoC, 2)) + "%")

    print("Battery Charge (" + estimate3Name + "): " + str(round(displaySoC, 2)) + "%")

    print("Battery Charge (" + estimate4Name + "): " + str(round(displaySoC, 2)) + "%")

    print("Battery Charge (" + estimate5Name + "): " + str(round(displaySoC, 2)) + "%")

    print("------------------------------------------")

    csvLine = "0.0" + "," + str(round(displaySoC, 2)) + "," + str(round(displaySoC, 2)) + "," + \
              str(round(displaySoC, 2)) + "," + str(round(displaySoC, 2)) + "," + str(round(displaySoC, 2)) + "\n"

    output_file.write(csvLine)

    # Initialize the state variable with an initial SoC estimate
    lastState1 = np.array([[initialSoC], [initialCurrent]])
    lastState2 = np.array([[initialSoC], [initialCurrent]])
    lastState3 = np.array([[initialSoC], [initialCurrent]])
    lastState4 = np.array([[initialSoC], [initialCurrent]])
    lastState5 = np.array([[initialSoC], [initialCurrent]])

    # Initialize the variance matrix, using predetermined values
    # This was originally 0.09, but I increased it to 0.5 to allow the Kalman Filter to automatically
    # choose the correct value
    lastVariance1 = np.array([[.5, 0], [0, .5]])
    lastVariance2 = np.array([[.5, 0], [0, .5]])
    lastVariance3 = np.array([[.5, 0], [0, .5]])
    lastVariance4 = np.array([[5.0, 0], [0, 5.0]])
    lastVariance5 = np.array([[5.0, 0], [0, 5.0]])

    # Initialize the observation matrix
    observationMatrix = np.array([[1, 0], [0, 1]])

    # Initialize the observation noise matrix
    observationNoise1 = np.array([[.25, 0], [0, .25]])
    observationNoise2 = np.array([[.25, 0], [0, .25]])
    observationNoise3 = np.array([[.25, 0], [0, .25]])
    observationNoise4 = np.array([[.2, 0], [0, .2]])
    observationNoise5 = np.array([[.2, 0], [0, .2]])

    # Initialize the transformation matrix
    totalCoulombs = 10800
    # Updated by Arduino
    deltaT = 0
    transformationMatrix = np.array([[1, -1 * (deltaT / totalCoulombs)*100], [0, 1]])

    # ESTIMATE
    # --------

    # Creates a non stop loop until the user stops data collection
    operate = True
    # Gets various initial times to control measurement frequencies
    dataStartTime = time.time()
    ocvStartTime = time.time()
    longCurrentStartTime = time.time()
    doLongCurrentUpdate = False
    extraTimeReference = 0
    extraTime = 0

    # Operate the Kalman Filter until shut off
    while operate:

        # Get an initial time to determine when to measure current
        startTime = time.time()

        # Delay 10 seconds to avoid over-use of CPU and energy
        time.sleep(10)

        # Update the deltaT value in the transformation matrix for the current period
        transformationMatrix = updateTransformationMatrix(time.time() - startTime + extraTime)

        # Estimate the state
        stateEstimate1 = np.matmul(transformationMatrix, lastState1)
        stateEstimate2 = lastState2
        stateEstimate3 = np.matmul(transformationMatrix, lastState3)
        stateEstimate4 = np.matmul(transformationMatrix, lastState4)

        # Only update estimate 5 if it has been 30 seconds
        if time.time() - longCurrentStartTime > 30:
            doLongCurrentUpdate = True
            transformationMatrixLong = updateTransformationMatrix(time.time() - longCurrentStartTime + extraTime)
            stateEstimate5 = np.matmul(transformationMatrixLong, lastState5)
        else:
            stateEstimate5 = lastState5

        # Estimate the variance
        varianceEstimate1 = np.matmul(np.matmul(transformationMatrix, lastVariance1), np.linalg.inv(transformationMatrix))
        varianceEstimate3 = np.matmul(np.matmul(transformationMatrix, lastVariance3), np.linalg.inv(transformationMatrix))
        varianceEstimate4 = np.matmul(np.matmul(transformationMatrix, lastVariance4), np.linalg.inv(transformationMatrix))
        # Only update variance for estimate 5 if it has been 30 seconds
        if doLongCurrentUpdate:
            varianceEstimate5 = np.matmul(np.matmul(transformationMatrixLong, lastVariance5),
                                          np.linalg.inv(transformationMatrixLong))
        else:
            varianceEstimate5 = lastVariance5

        # Reset the extra time variable, which measures the unaccounted for processing delays as Python executes
        extraTime = 0
        extraTimeReference = time.time()

        # Measure current to update the state estimate current
        arduino.write(str.encode(CURRENT_CODE))
        while arduino.inWaiting() < 0:
            time.sleep(0.1)
        arduinoData = readSerialLine(arduino)
        stateCurrent = float(arduinoData)

        # Update the current stored in the last state
        stateEstimate1[1] = stateCurrent
        stateEstimate3[1] = stateCurrent
        stateEstimate4[1] = stateCurrent
        if doLongCurrentUpdate:
            stateEstimate5[1] = stateCurrent

        # Update the last state to current state estimate
        lastState1 = stateEstimate1
        lastState3 = stateEstimate3
        lastState4 = stateEstimate4
        lastState5 = stateEstimate5

        # Update the last variance to current variance estimate
        lastVariance1 = varianceEstimate1
        lastVariance3 = varianceEstimate3
        lastVariance4 = varianceEstimate4
        lastVariance5 = varianceEstimate5

        # If 10 minutes has elapsed, get an OCV measurement and run the full KF
        if time.time() - ocvStartTime > 600:

            # MEASURE
            # -------
            # Measure the current OCV
            arduino.write(str.encode(OCV_CODE))
            while arduino.inWaiting() < 0:
                time.sleep(0.1)
            arduinoData = readSerialLine(arduino)
            measuredVoltage = float(arduinoData) / 2  # divide by 2 since SoC(OCV) function is for 1 cell's OCV

            # Calculate the measured SoC from OCV
            measuredSoC = getSoCFromOCV(measuredVoltage)

            # Use the measured current from most recent estimated state update
            measuredCurrent = stateCurrent

            # Create the state measurement matrix
            stateMeasurement = np.array([[measuredSoC], [measuredCurrent]])

            # CALCULATE
            # ---------

            # Calculate the measurement residual vector
            measurementResidual3 = stateMeasurement - stateEstimate3
            measurementResidual4 = stateMeasurement - stateEstimate4
            measurementResidual5 = stateMeasurement - stateEstimate5

            # Calculate the residual variance matrix
            residualVariance3 = varianceEstimate3 + observationNoise3
            residualVariance4 = varianceEstimate4 + observationNoise4
            residualVariance5 = varianceEstimate5 + observationNoise5

            # Calculate the Kalman gain
            kalmanGain3 = np.matmul(varianceEstimate3, np.linalg.inv(residualVariance3))
            kalmanGain4 = np.matmul(varianceEstimate4, np.linalg.inv(residualVariance4))
            kalmanGain5 = np.matmul(varianceEstimate5, np.linalg.inv(residualVariance5))

            # UPDATE
            # ------

            # Update the state
            lastState2 = stateMeasurement
            lastState3 = stateEstimate3 + np.matmul(kalmanGain3, measurementResidual3)
            lastState4 = stateEstimate4 + np.matmul(kalmanGain4, measurementResidual4)
            lastState5 = stateEstimate5 + np.matmul(kalmanGain5, measurementResidual5)

            # Update the variance
            lastVariance3 = np.matmul((np.identity(2) - kalmanGain3), varianceEstimate3)
            lastVariance4 = np.matmul((np.identity(2) - kalmanGain4), varianceEstimate4)
            lastVariance5 = np.matmul((np.identity(2) - kalmanGain5), varianceEstimate5)

            # Reset the clock for periodic OCV measurement
            ocvStartTime = time.time()

        # Reset long current clock (for estimate5 which only updates every 30 seconds
        if doLongCurrentUpdate:
            longCurrentStartTime = time.time()
            doLongCurrentUpdate = False

        # Print the results for the user to follow in real time
        displaySoC1 = float(lastState1[0])
        print("Battery Charge (" + estimate1Name + "): " + str(round(displaySoC1, 2)) + "%")

        displaySoC2 = float(lastState2[0])
        print("Battery Charge (" + estimate2Name + "): " + str(round(displaySoC2, 2)) + "%")

        displaySoC3 = float(lastState3[0])
        print("Battery Charge (" + estimate3Name + "): " + str(round(displaySoC3, 2)) + "%")

        displaySoC4 = float(lastState4[0])
        print("Battery Charge (" + estimate4Name + "): " + str(round(displaySoC4, 2)) + "%")

        displaySoC5 = float(lastState5[0])
        print("Battery Charge (" + estimate5Name + "): " + str(round(displaySoC5, 2)) + "%")

        print("------------------------------------------")

        # Write the data to a .csv file
        csvLine = str(round((time.time() - dataStartTime), 2)) + "," + str(round(displaySoC1, 2)) + "," + \
                  str(round(displaySoC2, 2)) + "," + str(round(displaySoC3, 2)) + "," + str(round(displaySoC4, 2)) \
                  + "," + str(round(displaySoC5, 2)) + "\n"

        output_file.write(csvLine)

        # Calculate the time it took Python to run a Kalman Filter iteration after the transformation matrix was set
        extraTime = time.time() - extraTimeReference


main()
