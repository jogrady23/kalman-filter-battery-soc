# DESCRIPTION
# -----------
# This program
#     - takes a .csv file containing an nx2 table of measurement number vs Open Circuit Voltage (OCV) readings
#     - turns it into a normalized State of Charge (SoC) vs OCV graph
#     - then fits an nth order polynomial function to the data, using the min chi-square order function
#
# The modeled nth order polynomial can be used for implementing a Kalman Filter, which is my ultimate goal here
#
# CONTACT
# -------
# For inquiries, contact jogrady@usc.edu
#
# Jack O'Grady
# © 2019

# IMPORTS
# Used for plotting graphs and creating the nth order polynomial fit
import numpy as np
import matplotlib.pyplot as plt

# FUNCTIONS OVERVIEW (detailed comments above each function)
# ----------------------------------------------------------
#     - csvToList: turns an nx2 csv file into a list of [[measurement, OCV], ... ]
#     - findAnomalyIndex: finds any noisy measurement points and returns the index in the measurement list
#     - fixSingleAnomaly: corrects a noisy data point by using the average of 2 adjacent measurements
#     - flipList: reverses the order a list
#     - createArray: turns two lists into a np.array
#     - createSpacedList: similar to range, except it can use non-int spacing
#     - getPolyFitValues: returns the modeled y-values list of an nth order polynomial fit using the experimental
#                         x-values list
#     - getChiSquaredValue: finds the chi-squared value given experimental and modeled values
#     - findOptimalOrderFit: finds the optimal order n of a polynomial fit using minimum chi-square analysis
#     - printFittingResults: prints the optimal order n, chi-squared value of the fit, and coefficients of
#                            the polynomial fit


# Used to open a 2 column csv file containing measurement number and cell voltage
# input: an nx2 .csv file of measurement number and Open Circuit Voltage (OCV)
# returns: a list of [[column1, column2], ... ]
def csvToList(fileName):
    dataFile = open(fileName, 'r')
    unformattedData = dataFile.readlines()
    formattedData = list()
    # get rid of column titles
    del unformattedData[0]
    # split the data by column, put in a formatted data list
    for line in unformattedData:
        currentLine = line.split(',')
        measurementNumber = int(currentLine[0])
        voltage = float(currentLine[1])
        formattedData.append([measurementNumber, voltage])
    dataFile.close()
    return formattedData


# Finds noisy data points in the experimental measurements of the cell's OCV
# input: a list with x,y values in each list object (the output of csvToList)
# returns: the indices of any anomalies, stored in a sequential list
def findAnomalyIndex(dataList):
    anomalyIndexList = list()
    for i in range(len(dataList)):
        if i != 0:
            if dataList[i][1] - dataList[i-1][1] > .01:
                anomalyIndexList.append(i)
    return anomalyIndexList


# Corrects a noisy data point by taking the average of the 2 adjacent data points
# input: the index of the anomaly (int), and list of x,y values in each list object (the output of csvToList)
# returns: none
def fixSingleAnomaly(anomalyIndex, dataSet):
    surroundingAverage = abs((dataSet[anomalyIndex + 1][1] + dataSet[anomalyIndex - 1][1])/2)
    # print(surroundingAverage)
    dataSet[anomalyIndex][1] = surroundingAverage
    # print(dataSet[anomalyIndex][0])


# Reverses the order of a list
# input: any list
# returns: the reversed-order list
def flipList(inputList):
    flippedList = list()
    for i in range(len(inputList)):
        flippedList.append(inputList[len(inputList) - i - 1])
    return flippedList


# Combines two lists to form a np.array
# inputs: two lists of x and y values
# returns: the np.array of the x and y values
def createArray(listX, listY):
    newList = list()
    for i in range(len(listX)):
        newList.append([listX[i], listY[i]])
    return np.array(newList)


# Creates a list of values between min and max with defined spacing, similar to range but can do non-int spacing
# inputs: the min value, the max value, and the spacing
# returns: a list of values between min and max separated by spacing
def createSpacedList(min, max, spacing):
    currentVal = min
    spacedList = list()
    i = 0
    while currentVal < max:
        currentVal = min + (i * spacing)
        spacedList.append(currentVal)
        i += 1
    return spacedList


# Creates a list of the modeled nth order polynomial fit values
# input: order n of the polynomial fit, list of experimental x values, list of experimental y values
# returns: a list of the modeled y-values using the polynomial fit
def getPolyFitValues(order, xList, yList):
    coefficients = np.polyfit(xList, yList, order)
    modeledValues = list()
    for x in xList:
        yVal = 0
        i = 0
        while order - i >= 0:
            yVal += coefficients[i] * x ** (order - i)
            i += 1
        modeledValues.append(yVal)
    return modeledValues


# Finds the chi-square value for experimental and modeled values
# inputs: list of experimental x-values, list of experimental y-values, list of modeled y-values
# returns: the chi-squared value
def getChiSquaredValue(experimentalValues, modeledValues):
    # Calculate the chi-squared value
    totalChiSquared = 0
    for i in range(len(experimentalValues)):
        expectedValue = modeledValues[i]
        observedValue = experimentalValues[i]
        if expectedValue != 0:
            totalChiSquared += abs(((observedValue - expectedValue)**2) / expectedValue)
        elif abs(observedValue - expectedValue) <= 0.000001:
            totalChiSquared += 0
        else:
            totalChiSquared += (observedValue + expectedValue)**2
    return totalChiSquared


# Finds the optimal order n of a polynomial fit using minimum chi-square analysis
# inputs: a list of experimental x values (OCV measurements) and list of experimental y values (normalized SoC)
# returns: a list object of [order n, chi-squared value of the nth order fit], where n is the optimal order fit
def findOptimalOrderFit(xValues, yValues):
    # Only checks order n=1:8 to minimize compute time
    n = 1
    chiSquaredResults = list()
    while n <= 8:
        currentChiSquared = getChiSquaredValue(yValues, getPolyFitValues(n, xValues, yValues))
        chiSquaredResults.append([n, currentChiSquared])
        n += 1
    # find the minimum order n
    minIndex = 0
    minChiSquared = 1000000000.0
    for i in range(len(chiSquaredResults)):
        if chiSquaredResults[i][1] < minChiSquared:
            minChiSquared = chiSquaredResults[i][1]
            minIndex = i
    # the returned object is of the form [order n, chi-squared value]
    return chiSquaredResults[minIndex]


# Prints the optimal order n, chi-squared value of the fit, and coefficients of the polynomial fit
# inputs: the chiSquaredResults output of findOptimalOrderFit(), a list of experimental x values (OCV measurements),
#         and list of experimental y values (normalized SoC)
# returns: none
def printFittingResults(chiSquaredResults, xValues, yValues):
    # prints the optimal order n
    print("Optimal Order Fit:", chiSquaredResults[0])
    # prints the chi-squared value of the fit
    print("Chi-Squared Value:", chiSquaredResults[1])
    # prints the coefficients of the optimal fit
    print("Coefficients:")
    coefficientList = np.polyfit(xValues, yValues, chiSquaredResults[0])
    order = chiSquaredResults[0]
    for i in range(len(coefficientList)):
        print("\tx^" + str(order - i) +":", coefficientList[i])


# Overview of Main
#    - Turn .csv data into a list
#    - Find the location of any anomalies/noisy data points
#    - Update the data lists to reflect the corrected noisy data points
#    - Turn the OCV vs Measurement Number Data into SoC vs OCV
#        • Flip order of data so 4.2V == 100%
#        • Normalize measurement numbers to SoC values on a scale of 0 to 100%
#    - Find the optimal order n of a polynomial fit of the data using minimum chi-square analysis
#    - Print the optimal order n, chi-squared value of the fit, and coefficients of the polynomial fit
#    - Plot the SoC vs OCV graph with the polynomial fit superimposed
def main():

    # Get the data as a list from an nx2 .csv file
    batteryData1 = csvToList("Samsung_18650_set2_run_1.csv")

    # Separate measurement numbers and OCV readings into separate lists
    measurementNumbers1 = list()
    voltageReading1 = list()
    for data in batteryData1:
        measurementNumbers1.append(data[0])
        voltageReading1.append(data[1])

    # Find anomalies - used if discharge is interrupted or there are isolated noisy data points
    anomalies = findAnomalyIndex(batteryData1)

    # Fix any anomalies
    for anomaly in anomalies:
        fixSingleAnomaly(anomaly - 1, batteryData1)

    # Refresh the lists with the corrected noisy data points
    measurementNumbers1 = list()
    voltageReading1 = list()
    for data in batteryData1:
        measurementNumbers1.append(data[0])
        voltageReading1.append(data[1])

    # Flip the order of the lists to allow for the SoC vs OCV plot
    measurementsAdjusted = flipList(measurementNumbers1)
    voltageReadingFinal = flipList(voltageReading1)

    # Normalize the data measurement numbers on a 0 to 100% scale
    numMeasurements = len(measurementsAdjusted)
    increment = 100.0/numMeasurements
    normalizedSoC = list()
    tempI = 0
    while tempI < numMeasurements:
        normalizedSoC.append(tempI*increment)
        tempI += 1

    # ----------------------------------------------------------------------------
    # At this point, normalizedSoC is the Y-Value list
    # At this point, voltageReadingFinal is the anomaly-corrected X-Value list
    # ----------------------------------------------------------------------------

    # Finds the optimal order n of a polynomial fit of the data using minimum chi-square analysis
    optimalFitValues = findOptimalOrderFit(voltageReadingFinal, normalizedSoC)
    optimalOrder = optimalFitValues[0]

    # Creates a list of modeled SoC values using the optimal polynomial fit, used for plotting
    modeledValues = getPolyFitValues(optimalOrder, voltageReadingFinal, normalizedSoC)

    # Prints the optimal order n, chi-squared value of the fit, and coefficients of the polynomial fit
    print("\nFITTING RESULTS")
    print("---------------")
    printFittingResults(optimalFitValues, voltageReadingFinal, normalizedSoC)

    # PLOTTING
    # set showRawData = True to view raw data
    # set showRawData = False to just view the fitted data
    showRawData = False

    if showRawData:
        # Plots the raw OCV vs measurement number data
        plt.subplot(1, 2, 1)
        plt.plot(measurementNumbers1, voltageReading1, label='Raw Data')
        plt.legend(loc='best')
        plt.ylabel('Cell Voltage (V)')
        plt.xlabel('Measurement Number')
        plt.title("Raw Data - Open Circuit Voltage (OCV)\nvs Measurement Number", fontweight='bold')
        plt.grid(True)
        # Plots the experimental SoC vs OCV
        plt.subplot(1, 2, 2)
        plt.plot(voltageReadingFinal, normalizedSoC, label='Experimental Data')
        # Plots the modeled values using the polynomial fit for SoC vs OCV
        plt.plot(voltageReadingFinal, modeledValues, label='Polynomial Fit')
        plt.legend(loc='best')
        plt.ylabel('State of Charge (%)')
        plt.xlabel('Cell Voltage (V)')
        plt.title("Battery State of Charge (SoC) vs\nOpen Circuit Voltage (OCV)", fontweight='bold')
        plt.grid(True)
        # Adjust spacing of subplots
        plt.subplots_adjust(wspace=0.35)
    else:
        # Plots the experimental SoC vs OCV
        plt.plot(voltageReadingFinal, normalizedSoC, label='Experimental Data')
        # Plots the modeled values using the polynomial fit for SoC vs OCV
        plt.plot(voltageReadingFinal, modeledValues, label='Polynomial Fit')
        plt.legend(loc='best')
        plt.ylabel('State of Charge (%)')
        plt.xlabel('Cell Voltage (V)')
        plt.title("Battery State of Charge (SoC) vs\nOpen Circuit Voltage (OCV)", fontweight='bold')
        plt.grid(True)
        # Adjust spacing of subplots
        plt.subplots_adjust(wspace=0.35)

    plt.show()


main()
