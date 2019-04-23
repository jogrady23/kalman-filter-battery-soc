# DESCRIPTION
# -----------
# • This program plots various estimates for the state of charge (SoC) of a battery
#
# CONTACT
# -------
# For inquiries, contact jogrady@usc.edu
#
# Jack O'Grady
# © 2019

# IMPORTS
# -------
# Used for plotting graphs and creating the nth order polynomial fit
import numpy as np
import matplotlib.pyplot as plt


# FUNCTIONS
# ---------


# Used to open a 6 column csv file containing time measurements and SoC estimates
# Input: an nx6 .csv file of [time, coulomb count, OCV, KF, KF, KF]
# Returns: a list of [[columnTitles], [[time, coulomb count, OCV, KF, KF, KF], ...]]
def csvToList(fileName):
    dataFile = open(fileName, 'r')
    unformattedData = dataFile.readlines()
    formattedData = list()
    # get column titles
    titleLineFunction = unformattedData[0].split(',')
    # ignore column titles for rest of data
    relevantData = unformattedData[1:]
    # split the data by column, put in a formatted data list
    for line in relevantData:
        currentLine = line.split(',')
        time = float(currentLine[0])
        estimate1 = float(currentLine[1])
        estimate2 = float(currentLine[2])  # holds the OCV SoC estimate data
        estimate3 = float(currentLine[3])
        estimate4 = float(currentLine[4])
        estimate5 = float(currentLine[5])
        formattedData.append([time, estimate1, estimate2, estimate3, estimate4, estimate5])
    dataFile.close()
    return [titleLineFunction, formattedData]


# This function selects all the unique values from the OCV data list, since the OCV is only measured once every
# 600 seconds, but a the value is still recorded at every time interval
# Input: the SoC estimates from OCV list as [[time, SoC from OCV estimate], ...]
# Returns: a list of [[time, unique SoC], ...]
def fixOCVData(ocvList, timeList):
    fixedOCVList = list()
    lastOCV = -100.0
    for i in range(len(ocvList)):
        # only chooses a value if it is different from the last
        if ocvList[i] != lastOCV:
            fixedOCVList.append([timeList[i], ocvList[i]])
            lastOCV = ocvList[i]
    return fixedOCVList


# Subtracts the SoC estimate of each method from the SoC estimate of the OCV measurements, then divides that quantity
# by the current SoC value from the OCV method
# Input: all estimates that are being analyzed
# Returns: A list of the relative differences for each time step [[time, difEstimate1, ...], ... ]
def getRelDifFromOCV(timeList, ocvList, list1, list2, list3):
    difList = list()
    lastOCV = -100.0
    for i in range(len(ocvList)):
        if ocvList[i] !=lastOCV:
            difList1 = ((list1[i] - ocvList[i])/ocvList[i])*100
            difList2 = ((list2[i] - ocvList[i]) / ocvList[i]) * 100
            difList3 = ((list3[i] - ocvList[i]) / ocvList[i]) * 100
            difList.append([timeList[i], difList1, difList2, difList3])
            lastOCV = ocvList[i]
    return difList


# Subtracts the SoC estimate of each method from the SoC estimate of the OCV measurements
# Input: all estimates that are being analyzed
# Returns: A list of the absolute differences for each time step [[time, difEstimate1, ...], ... ]
def getAbsDifFromOCV(timeList, ocvList, list1, list2, list3, list4):
    difList = list()
    lastOCV = -100.0
    for i in range(len(ocvList)):
        if ocvList[i] !=lastOCV:
            difList1 = ((list1[i] - ocvList[i]))
            difList2 = ((list2[i] - ocvList[i]))
            difList3 = ((list3[i] - ocvList[i]))
            difList4 = ((list4[i] - ocvList[i]))
            difList.append([timeList[i], difList1, difList2, difList3, difList4])
            lastOCV = ocvList[i]
    return difList


# MAIN
# ----------

def main():

    # Get the data as a list from an nx6 .csv file
    kalmanData1 = csvToList("Kalman_Filter_Comparison_5column_extra_time.csv")

    # Get estimate names from column titles
    titleLine = kalmanData1[0]
    estimate1Name = titleLine[1]  # since titleLine[0] is the time
    estimate2Name = titleLine[2]
    estimate3Name = titleLine[3]
    estimate4Name = titleLine[4]
    estimate5Name = titleLine[5].strip('\n')

    # Separate measurement numbers and OCV readings into separate lists
    # Create the lists to be filled
    timeMeasurements = list()
    estimate1Measurements = list()
    estimate2Measurements = list()
    estimate3Measurements = list()
    estimate4Measurements = list()
    estimate5Measurements = list()
    # Fill the lists
    for data in kalmanData1[1]:
        timeMeasurements.append(data[0])
        estimate1Measurements.append(data[1])
        estimate2Measurements.append(data[2])
        estimate3Measurements.append(data[3])
        estimate4Measurements.append(data[4])
        estimate5Measurements.append(data[5])

    # PLOTTING
    # ------------
    # OCVSetting Options
    # 1 = SoC Estimate vs Time, show step OCV
    # 2 = SoC Estimate vs Time, no step OCV
    # genSetting Options
    # 1 = SoC Estimate vs Time
    # 2 = SoC Estimate - Actual vs Time
    # 3 = Both

    # Set the OCVSetting choice
    OCVSetting = 2
    # Set the genSetting choice
    genSetting = 3

    # Creates data points for the "actual" OCV SoC estimate vs time by choosing unique SoC values from the OCV method
    newOCV = fixOCVData(estimate2Measurements, timeMeasurements)
    newOCVTime = list()
    newOCVPoints = list()
    for data in newOCV:
        newOCVTime.append(data[0])
        newOCVPoints.append(data[1])

    # Plots the SoC vs time for each method
    if genSetting != 2:
        if genSetting == 3:
            plt.subplot(1, 2, 1)
        plt.plot(timeMeasurements, estimate1Measurements, label=estimate1Name, color='#02ECFF')
        if OCVSetting == 1:
            plt.plot(timeMeasurements, estimate2Measurements, label=estimate2Name, color="b")
        plt.plot(timeMeasurements, estimate3Measurements, label=estimate3Name, color='#7D3535')
        plt.plot(timeMeasurements, estimate4Measurements, label=estimate4Name, color='#FF8102')
        plt.plot(timeMeasurements, estimate5Measurements, label=estimate5Name, color='r')
        plt.plot(newOCVTime, newOCVPoints, label='\"True\" State of Charge', color='#000000')
        plt.legend(loc='best')
        plt.ylabel('State of Charge (%)')
        plt.xlabel('Time (s)')
        plt.title("Battery State of Charge vs Time\nUsing Various Estimation Methods\n(OCV Measurement Frequency: 600s)", fontweight='bold')
        plt.grid(True)

    # Calculates the difference between a SoC estimate and the SoC via OCV method
    wholeDifList = getAbsDifFromOCV(timeMeasurements, estimate2Measurements, estimate1Measurements,
                                 estimate3Measurements, estimate4Measurements, estimate5Measurements)
    # Create lists to be filled
    timeDifList = list()
    estimate1Dif = list()
    estimate3Dif = list()
    estimate4Dif = list()
    estimate5Dif = list()
    # Fill the lists
    for data in wholeDifList:
        timeDifList.append(data[0])
        estimate1Dif.append(data[1])
        estimate3Dif.append(data[2])
        estimate4Dif.append(data[3])
        estimate5Dif.append(data[4])

    # Plots the absolute difference between a SoC estimate and the SoC via OCV method
    if genSetting != 1:
        if genSetting == 3:
            plt.subplot(1, 2, 2)
        plt.plot(timeDifList, estimate1Dif, label=estimate1Name, color='#02ECFF')
        plt.plot(timeDifList, estimate3Dif, label=estimate3Name, color='#7D3535')
        plt.plot(timeDifList, estimate4Dif, label=estimate4Name, color='#FF8102')
        plt.plot(timeDifList, estimate5Dif, label=estimate5Name, color='r')
        plt.legend(loc='best')
        plt.ylabel('% Difference Between Estimate and OCV Measurement')
        plt.xlabel('Time (s)')
        plt.title("Divergence of the Battery State of Charge\nEstimate from Measurement vs Time\n" +
                  "For Various Estimation Methods", fontweight='bold')
        plt.grid(True)

    # Adjust spacing of subplots
    if genSetting == 3:
        plt.subplots_adjust(wspace=0.35)

    # Show the plots
    plt.show()


main()
