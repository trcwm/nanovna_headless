#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import rfft, rfftfreq
from scipy.fft import fft, fftfreq
from scipy import signal

data = np.array([
-3436,
  -6505,
  -5918,
  -2470,
  579,
  3948,
  6550,
  5053,
  1558,
  -1420,
  -4864,
  -6833,
  -4635,
  -1173,
  1822,
  5327,
  6469,
  3634,
  332,
  -2725,
  -6103,
  -6403,
  -3178,
  -2,
  3219,
  6304,
  5661,
  2225,
  -817,
  -4159,
  -6766,
  -5322,
  -1803,
  1180,
  4661,
  6608,
  4370,
  931,
  -2054,
  -5518,
  -6706,
  -3910,
  -577,
  2502,
  5877,
  6139,
  2927,
  -238
])

#data = np.append(data, np.zeros(4096-len(data)))
#data = signal.resample(data, len(data)*10)

X = rfft(data)
Xf = rfftfreq(len(data), 1 / 48000)

n = np.linspace(0, len(data), len(data), endpoint=False)
print(n)

plt.figure(1)
plt.plot(n, data, n, -10000.0*np.sin(2.0*3.1415927*(n+0.5) * 5000/48000))
plt.plot(n, data)
plt.grid()
plt.title("NanoVNA RAW buffer waveform")

plt.figure(2)
plt.plot(Xf, 20*np.log10(np.abs(X)))
plt.grid()
plt.title("NanoVNA RAW buffer waveform spectrum")

nco = np.exp(-2j*3.1415027*n * 5000/48000)
dmix = np.multiply(data, nco)

plt.figure(3)
plt.plot(n, np.real(dmix), n, np.imag(dmix))
plt.grid()
plt.title("NanoVNA down-mixer RAW buffer")

X2  = np.fft.fftshift(fft(dmix))
Xf2 = np.fft.fftshift(fftfreq(len(dmix), 1 / 48000))

b = np.ones(len(data))/len(data)
w,h = signal.freqz(b,1, 4096, fs=48000)

plt.figure(4)
plt.plot(Xf2, 20*np.log10(np.abs(X2)), w, 100+20*np.log10(np.abs(h)))
plt.grid()
plt.title("NanoVNA down-mixed RAW buffer waveform spectrum")

I = np.sum(np.real(dmix))
Q = np.sum(np.imag(dmix))
print(I,Q)
print("angle = ", 180.0*np.arctan2(Q,I)/np.pi)

plt.show()
