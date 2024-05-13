import sounddevice as sd
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    # If you got "ValueError: No input device matching", that is because your PC name example device
    # differently from tested list below. Uncomment the next line to see full list and try to pick correct one
    #print(sd.query_devices())

    fs = 8000  		# Sample rate
    duration = 5000e-3   # Duration of recording

 
    device = 'Mikrofon (SooperDooper 4ch), Windows WDM-KS'


    myrecording = sd.rec(int(duration * fs), samplerate=fs, channels=2, dtype='int16', device=device)
    print('Waiting...')
    sd.wait()  # Wait until recording is finished
    print('Done!')

    time = np.arange(0, duration, 1 / fs)  # time vector
    plt.plot(time, myrecording)
    plt.xlabel('Time [s]')
    plt.ylabel('Amplitude')
    plt.title('MicNode 4 Channel')
    plt.show()