# kalman-filter-battery-soc
A Kalman Filter that estimates the state of charge of 2 Li-ion cells

More background information available at https://www.jackogrady.me/battery-management-system/state-of-charge

## Hardware Overview
To run the Kalman Filter, you'll need:
  1. A computer running Python
  2. An Arduin Uno board
  3. The configured Kalman Filter circuit (shown in the link above)
  4. Two 18650 Li-ion cells

To view results and data analysis, you'll need:
  1. A computer running Python

## Code Overview

### Running the Kalman Filter
• kalman_filter_operation.py runs the Kalman Filter when connected to Arduino
• Arduino_kalman_filter.ino should be uploaded onto an Arduino Uno (which

### Analyzing Data
• kalman_filter_data_analysis.py
• Kalman_Filter_Experimental_Data.csv (a sample run of data)

