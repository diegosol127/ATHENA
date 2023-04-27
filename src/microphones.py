import smbus2
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
import scipy.integrate as integrate
import scipy.special as special

bus = smbus2.SMBus(1)
pcf8591_address = 0x48

def read_pcf8591(channel):
    bus.write_byte(pcf8591_address, channel)
    bus.read_byte(pcf8591_address)  # Discard the first reading (buffered value)
    return bus.read_byte(pcf8591_address)

def sample_audio_data(channel):
    data = read_pcf8591(channel)
    return data

def butter_bandpass(lowcut, highcut, fs, order=5):
    return butter(order, [lowcut, highcut], fs=fs, btype="band")

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

class MovingAverage:
    def __init__(self):
        self.buffer = []
        self.max_buffer_length = 1

    def update(self, x):
        self.buffer.append(x)
        if len(self.buffer) > self.max_buffer_length:
            self.buffer = self.buffer[1:]
        return np.mean(self.buffer)

n_samples = 5000

try:
    print("Started Sampling...")
    t = np.zeros(n_samples,dtype=float)
    y = np.zeros(n_samples,dtype=float)
    # right = np.zeros_like(t)
    right = np.zeros_like(t)
    left = np.zeros_like(t)
    front = np.zeros_like(t)
    back = np.zeros_like(t)
    t0 = time.time()
    for i in range(len(t)):
        # x1[i] = sample_audio_data(2)
        # print(bus.read_i2c_block_data(pcf8591_address, 2, 16))
        right[i] = sample_audio_data(2)
        left[i] = sample_audio_data(0)
        # front[i] = sample_audio_data(1)
        # back[i] = sample_audio_data(3)
        # right.append(bus.read_byte_data(pcf8591_address, 3))
        # left.append(bus.read_byte_data(pcf8591_address, 1))
        # front.append(bus.read_byte_data(pcf8591_address, 2))
        # back.append(bus.read_byte_data(pcf8591_address, 0))
        # print(sample_audio_data(3))
        dt = time.time() - t0
        t[i] = dt
        # time.sleep(0.001)
    t = t - t[0]
    print("Sampling ended")
except KeyboardInterrupt:
    pass

X = np.fft.fft(right)
n = np.arange(len(X))

t = np.linspace(0, t[-1], num=len(right))


plt.figure(1)
plt.plot(t,right,'-')
plt.plot(t,left,'-')
plt.plot(t,front,'-')
plt.plot(t,back,'-')
plt.legend(["Right", "Left", "Front", "Back"])
# plt.figure(2)
plt.show()

print(X)
time_step = t[-1]/len(right)
print(round(time_step, 10))

freqs = np.fft.fftfreq(n_samples, time_step)
idx = np.argsort(freqs)

plt.plot(freqs[idx], X[idx])
plt.show()

fs = 1/time_step

lowcut = 80
highcut = 180

right_filtered = butter_bandpass_filter(right, lowcut, highcut, fs)
left_filtered = butter_bandpass_filter(left, lowcut, highcut, fs)
front_filtered = butter_bandpass_filter(front, lowcut, highcut, fs)
back_filtered = butter_bandpass_filter(back, lowcut, highcut, fs)
X = np.fft.fft(right_filtered)
n = np.arange(len(X))

plt.plot(abs(right_filtered)**2, alpha=0.5)
plt.plot(abs(left_filtered)**2, alpha=0.5)
plt.plot(abs(front_filtered)**2, alpha=0.5)
plt.plot(abs(back_filtered)**2, alpha=0.5)
plt.legend(["Right", "Left", "Front", "Back"])
plt.show()

def moving_average(a, n=1000):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n-1:]/n

right_ma = moving_average(abs(right_filtered))-moving_average(abs(left_filtered))

print("Integral:", np.trapz(right_ma, dx=1))

plt.plot(moving_average(abs(right_filtered))-moving_average(abs(left_filtered)), alpha=0.5)
plt.plot(moving_average(abs(front_filtered))-moving_average(abs(back_filtered)), alpha=0.5)
# plt.plot(moving_average(abs(left_filtered)**2), alpha=0.5)
# plt.plot(moving_average(abs(x3_filtered)**2), alpha=0.5)
plt.legend(["Right", "Front"])
plt.hlines([0], 0, n_samples)
plt.show()

mean_of_right_v_left = np.max(moving_average(abs(right_filtered))-moving_average(abs(left_filtered)))

mean_of_front_v_back = np.mean(abs(front_filtered)**2-abs(back_filtered)**2)

right_integraral = np.trapz(right_ma, dx=1)
front_norm = mean_of_front_v_back

print(right_integraral)
print(front_norm)

# print(np.rad2deg(np.arctan2(front_norm, right_norm))+90)

if abs(right_integraral) > 1000:
    if right_integraral > 0:
        print("Turn right")
    else:
        print("Turn left")
else:
    print("Don't Turn")

if abs(front_norm) > 10:
    if front_norm > 0:
        print("Go forward")
    else:
        print("Go backwards")

# freqs = np.fft.fftfreq(n_samples, time_step)
# idx = np.argsort(freqs)
# plt.plot(freqs[idx], X[idx])
# plt.show()

# n_samples = 1000

# t = np.arange(n_samples,dtype=float)
# y = list(np.zeros(n_samples,dtype=float))

# fig = plt.figure()
# ax = fig.add_subplot(111)
# line1, = ax.plot(t, y)


# try:
#     for i in range(len(t)):
#         y = sample_audio_data(1)
#         plt.scatter(i, y, c="b")
#         plt.pause(0.1)
# except Exception as e:
#     print("Error:". e)
#     pass

# plt.show()