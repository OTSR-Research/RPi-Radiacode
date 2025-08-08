import sys
import time
import numpy as np
import yaml
from radiacode import RadiaCode

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# Physics constants
rho_CsJ = 4.51  # g/cm³
m_sensor = rho_CsJ * 1e-3  # kg/cm³
keV2J = 1.602e-16
depositedE2doserate = keV2J * 3600 * 1e6 / m_sensor
depositedE2dose = keV2J * 1e6 / m_sensor


def plot_RC103Spectrum():
    global a0, a1, a2

    a0 = 0.17
    a1 = 2.42
    a2 = 0.0004

    def Chan2En(C):
        return a0 + a1 * C + a2 * C**2

    def En2Chan(E):
        c = a0 - E
        return (np.sqrt(a1**2 - 4 * a2 * c) - a1) / (2 * a2)

    restart_accumulation = True
    reset_device_spectrum = True
    reset_device_dose = True
    dt_wait = 1.0
    timestamp = time.strftime('%y%m%d-%H%M', time.localtime())
    filename = 'launch_data' + '_' + timestamp + '.yaml'
    print(filename)

    NHistory = 500
    run_time = 36000
    rate_history = np.zeros(NHistory)

    print(f'\n *==* script {sys.argv[0]} executing')
    print('    connecting via USB')

    rc = RadiaCode()
    serial = rc.serial_number()
    fw_version = rc.fw_version()
    status_flags = eval(rc.status().split(':')[1])[0]
    a0, a1, a2 = rc.energy_calib()
    rc.set_device_on(True)
    if reset_device_spectrum:
        rc.spectrum_reset()
    if reset_device_dose:
        rc.dose_reset()

    spectrum = rc.spectrum()
    counts0 = np.asarray(spectrum.counts)
    NChannels = len(counts0)
    Channels = np.asarray(range(NChannels)) + 0.5
    Energies = Chan2En(Channels)
    duration_s = spectrum.duration.total_seconds()

    print(f'### Found device with serial number: {serial}')
    print(f'    Firmware: {fw_version}')
    print(f'    Status flags: 0x{status_flags:x}')
    print(f'    Calibration coefficients: a0={a0:.6f}, a1={a1:.6f}, a2={a2:.6f}')
    print(f'    Number of spectrum channels: {NChannels}')
    print(f'    Spectrum accumulation since {spectrum.duration}')
    print('### Waiting for launch (non-zero acceleration)...')

    # Initialize MPU9250
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu=MPU9050_ADDRESS_68,
        bus=1,
        gfs=GFS_250,
        afs=AFS_2G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ)

    mpu.calibrate()
    mpu.configure()

    while True:
        ax, ay, az = mpu.readAccelerometerMaster()
        a_mag = (ax**2 + ay**2 + az**2)**0.5
        if a_mag > 0.5:  # Threshold to detect motion
            t_launch = time.time()
            print("Motion detected. Timer started.")
            break
        time.sleep(0.1)

    # Wait until t = 10 seconds
    print("Waiting until t = 10s to begin spectrum recording...")
    while time.time() - t_launch < 10:
        time.sleep(0.1)

    print("=== Starting spectrum recording at t = 10s ===")

    toggle = ['  \\ ', '  | ', '  / ', '  - ']
    itoggle = 0
    icount = -1
    total_time = 0
    previous_counts = counts0.copy()

    if restart_accumulation:
        counts = np.zeros(len(counts0))
        T0 = t_launch + 10
    else:
        counts = counts0.copy()
        T0 = t_launch + 10 - duration_s

    countsum0 = np.sum(counts)
    recorded = False

    try:
        while True:
            _t = time.time()
            elapsed = _t - T0

            if elapsed >= 500:
                print("\n=== Stopping spectrum recording at t = 500s ===")
                break

            icount += 1
            total_time = int(10 * elapsed) / 10
            spectrum = rc.spectrum()
            actual_counts = np.asarray(spectrum.counts)
            if not any(actual_counts):
                time.sleep(dt_wait)
                print(' accumulation time:', total_time, ' s', ' !!! waiting for data', end='\r')
                continue

            counts_diff = actual_counts - previous_counts
            previous_counts[:] = actual_counts
            counts += counts_diff
            countsum = np.sum(counts)
            rate = (countsum - countsum0) / dt_wait
            rate_history[icount % NHistory] = rate
            rate_av = countsum / total_time
            depE = np.sum(counts_diff * Energies)
            doserate = depE * depositedE2doserate / dt_wait
            deposited_energy = np.sum(counts * Energies)
            total_dose = deposited_energy * depositedE2dose
            av_doserate = deposited_energy * depositedE2doserate / total_time

            countsum0 = countsum

            print(
                toggle[itoggle],
                ' active:',
                total_time,
                's  ',
                f'counts: {countsum:.5g}, rate: {rate:.3g} Hz, dose: {doserate:.3g} µGy/h',
                '    (<ctrl>+c to stop)      ',
                end='\r',
            )
            itoggle = (itoggle + 1) % 4
            time.sleep(0.5)

        print('\n' + sys.argv[0] + ': exit after ', total_time, ' s of data accumulation ...')

    finally:
        print(22 * ' ' + '... storing data to yaml file ->  ', filename)
        d = dict(
            active_time=total_time,
            interval=dt_wait,
            total_dose=total_dose,
            average_dose_rate=av_doserate,
            average_rate=rate_av,
            rates=rate_history[: icount + 1].tolist()
            if icount < NHistory
            else np.concatenate((rate_history[icount + 1:], rate_history[: icount + 1])).tolist(),
            ecal=[a0, a1, a2],
            spectrum=counts.tolist(),
        )
        with open(filename, 'w') as f:
            f.write(yaml.dump(d, default_flow_style=None))

        for v in rc.data_buf():
            print(v.dt.isoformat(), v)

        time.sleep(2)
        rc.set_device_on(False)


if __name__ == '__main__':
    plot_RC103Spectrum()