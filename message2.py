import time
import csv
import numpy as np
from radiacode import RadiaCode
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# ==== CONFIG ====
ACCEL_THRESHOLD = 0.5  # Acceleration threshold before launch is detected (G-Force on Z-axis)
RECORD_DELAY = 10      # Delay before recording starts (in seconds)
RECORD_DURATION = 300  # total time we're recording for (in seconds)
OUTPUT_FILE = "/home/pi/launch_spectrum.csv"  # Save spectrum data to csv file 
# =================

# Radiacode Calibration Coefficients (MAKE SURE TO CALIBRATE BEFORE LAUNCH)
def chan_to_energy(chan, a0, a1, a2):
    return a0 + a1 * chan + a2 * chan**2

def main():
    # Initialize the MPU9250 Accelerometer (this YouTube vid explains: https://www.youtube.com/watch?v=vZ7yVSyqYRo)
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu=MPU9050_ADDRESS_68,
        bus=1,
        gfs=GFS_250,
        afs=AFS_2G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ
    )
    mpu.calibrate()
    mpu.configure()

# Iteratively check for the acceleration threshold to be met
    print("Waiting for motion...") # This line is just for debugging to make sure it works while testing on the monitor
    while True:
        ax, ay, az = mpu.readAccelerometerMaster()
        if az > ACCEL_THRESHOLD:  # Threshold for launch detection on Z-axis (CHANGE IF NEEDED)
            t_launch = time.time()
            print("Launch detected (Z-axis threshold exceeded). Timer started.") # Again, just for debugging
            break
        time.sleep(0.1) # Sampling Rate: checks acceleration every 0.1s

    # Initialize the RadiaCode (Inspired by Ryman's script)
    rc = RadiaCode()
    print(f"Device: {rc.serial_number()} FW: {rc.fw_version()}")
    a0, a1, a2 = rc.energy_calib()
    rc.set_device_on(True)
    rc.spectrum_reset()

    # Wait 10 seconds after launch before recording (to avoid detection of K-40 peak on ground)
    while time.time() - t_launch < RECORD_DELAY:
        time.sleep(0.1)

    # Spectrum recording starts until 300 seconds pass
    print("Recording spectrum...")  # Debugging message
    start_time = time.time()
    while time.time() - start_time < RECORD_DURATION:
        time.sleep(1)  

    # Get the final spectrum
    spectrum = rc.spectrum()
    counts = np.asarray(spectrum.counts)
    channels = np.arange(len(counts)) + 0.5 # The +0.5 just puts the channel number in the middle of the energy bin
    energies = chan_to_energy(channels, a0, a1, a2) # Convert channels to energies based on calibration coefficients

    # Save CSV File (Energy, Counts)
    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Energy_keV", "Counts"])
        for E, C in zip(energies, counts):
            writer.writerow([E, C])

    print(f"Spectrum saved to {OUTPUT_FILE}")
    rc.set_device_on(False)

main()