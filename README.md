# kalman-filter-battery-soc
A Kalman Filter that estimates the state of charge of 2 Li-ion cells

More information available at https://www.jackogrady.me/battery-management-system/state-of-charge

## Overview
Since a battery’s state of charge (SoC) cannot be directly measured, it is estimated using state variables of related characteristics—namely the battery’s open circuit voltage (OCV) and the current leaving or entering the cell; however, neither of these methods are sufficient on their own. The cell's OCV measurement is noisy and cannot be constantly measured as it requires disconnecting whatever device the battery is powering. Additionally, estimating the SoC by coulomb counting (measuring every charge leaving or entering the cell) gradually produces significant error over long time intervals. By dynamically combining these two measurements, though, a more accurate estimate can be achieved. Enter, the Kalman Filter.
