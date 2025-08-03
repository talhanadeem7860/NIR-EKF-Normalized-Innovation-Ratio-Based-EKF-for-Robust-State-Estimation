NIR-EKF: Robust State Estimation with an Adaptive EKF

Project Overview:
This repository provides a MATLAB implementation of a robust state estimation algorithm called NIR-EKF (Normalized Innovation Ratio-Based Extended Kalman Filter). It includes a simulation to directly compare the performance of the standard Extended Kalman Filter (EKF) against the more robust NIR-EKF.

Standard EKFs are highly sensitive to sensor measurement outliers, which can cause the filter to fail. The NIR-EKF solves this by performing a statistical check on each new measurement. If a measurement is flagged as an outlier, the filter adaptively reduces its reliance on it, preventing the state estimate from being corrupted. This results in significantly more stable and accurate tracking in noisy environments.

How to Run:

Open the project folder in MATLAB.

Open the respective .m file.

Click the "Run" button or press F5.

The script will print the final TRMSE values for both filters and generate the results plot.

Expected Results:

Running the simulation will generate a plot comparing the two filters. As shown, the standard EKF's estimate (red) diverges when faced with outliers, while the NIR-EKF (green) remains stable and accurately tracks the true state.
