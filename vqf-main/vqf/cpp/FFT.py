import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.fftpack import fft, ifft

def read_data(file_path_imu, file_path_mag):
    imu_data = []
    mag_data = []

    with open(file_path_imu, 'r') as file:
        for line in file:
            try:
                values = list(map(float, line.strip().split()))
                imu_data.append(values)
            except ValueError:
                print(f"Error parsing line: {line.strip()}")

    with open(file_path_mag, 'r') as file_mag:
        for line in file_mag:
            try:
                values = list(map(float, line.strip().split()))
                mag_data.append(values)
            except ValueError:
                print(f"Error parsing line: {line.strip()}")

    imu_df = pd.DataFrame(imu_data, columns=[
        'timestamp', 'index', 'roll', 'pitch', 'yaw', 'wx', 'wy', 'wz', 'ax', 'ay', 'az' , 'flag'
    ])
    imu_df['ax'] = imu_df['ax'] / 1000.0 * 9.81
    imu_df['ay'] = imu_df['ay'] / 1000.0 * 9.81
    imu_df['az'] = imu_df['az'] / 1000.0 * 9.81
    print(imu_df['az'])
    imu_df['wx'] = imu_df['wx'] / 100.0 * np.pi / 180.0
    imu_df['wy'] = imu_df['wy'] / 100.0 * np.pi / 180.0
    imu_df['wz'] = imu_df['wz'] / 100.0 * np.pi / 180.0

    mag_df = pd.DataFrame(mag_data, columns=['timestamp', 'x', 'y', 'z'])

    return imu_df, mag_df

def plot_fft(data, sample_rate, title):
    n = len(data)
    freqs = np.fft.fftfreq(n, d=1/sample_rate)
    fft_values = np.abs(fft(data))

    plt.figure(figsize=(10, 5))
    plt.plot(freqs[:n // 2], fft_values[:n // 2])
    plt.title(f"FFT of {title}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.grid()
    plt.show()

def butter_lowpass_filter(data, cutoff_freq, sample_rate, order=2):
    nyquist = 0.5 * sample_rate
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

def main():
    file_path_imu = "/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241220_100903/userdata/hj/log/sensor_data_alg/data/pool_bottom/imu.log"
    file_path_mag = "/home/zc/data/12_20/logs_ScubaX1ProMax_X9X45000061_20241220_100903/userdata/hj/log/sensor_data_alg/data/pool_bottom/mag.log"

    imu_df, mag_df = read_data(file_path_imu, file_path_mag)

    # Sample rate in Hz (assuming timestamps are in seconds)
    sample_rate = 100  # Adjust based on your data

    # Analyzing acceleration in x-direction
    acc_x = imu_df['wz'].values

    # Perform FFT and visualize
    plot_fft(acc_x, sample_rate, "Acceleration X")

    # Set cutoff frequency for low-pass filtering
    cutoff_freq = 0.5  # Hz

    # Apply Butterworth low-pass filter
    filtered_acc_x = butter_lowpass_filter(acc_x, cutoff_freq, sample_rate)

    # Perform IFFT for filtered data
    filtered_fft = fft(filtered_acc_x)
    reconstructed_signal = ifft(filtered_fft).real

    # Visualize results
    plt.figure(figsize=(12, 6))
    plt.plot(acc_x, label="Original Signal")
    plt.plot(filtered_acc_x, label="Filtered Signal", linestyle="--")
    plt.title("Original vs Filtered Signal")
    plt.xlabel("Sample Index")
    plt.ylabel("Acceleration (m/s^2)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
