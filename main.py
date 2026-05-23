import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"
UDP_PORT = 7450

N = 512
FS = 48000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind((UDP_IP, UDP_PORT))

plt.ion()

fig, ax = plt.subplots()

freqs = np.fft.rfftfreq(N, 1 / FS)

line, = ax.plot(freqs, np.zeros(len(freqs)))

ax.set_xlim(0, FS // 2)
ax.set_ylim(0, 15)

halfsamples = 0

while True:

    data, addr = sock.recvfrom(2048)

    length = len(data)


    samples = struct.unpack("<257i", data)

    samples = np.array(samples, dtype=np.float32)
    packetcounter = samples[0]
    samples = samples[1:257]

    if(packetcounter%2 == 0):
        halfsamples = samples
    else:
        samples = np.append(halfsamples, samples)
    
    if(packetcounter%2 == 1):
        
        samples /= 2147483648.0

        samples *= np.hanning(N)

        fft = np.fft.rfft(samples)

        mag = np.abs(fft)

        mag = 2/N * mag * 0.9 * np.sqrt(2)
        mag = mag / 0.0126
        #mag = mag / N

        hz1kindex = int(1000*N*2 / FS)

        print(f"1 kHz bin: {hz1kindex}, Value: {mag[hz1kindex]}")
        print(f"spl: {20*np.log10(mag[hz1kindex] / 20e-6)} dB")

        line.set_ydata(mag)

        plt.draw()
        plt.pause(0.001)