'''
Use these while working on achieving phase coherency.
'''
import numpy as np 
import scipy
import matplotlib.pyplot as plt
import math
from scipy import signal
from scipy.fftpack import fft, fftshift
from scipy.signal import blackmanharris
from termcolor import colored
from IPython import get_ipython
ipython = get_ipython()

def plot_complete(series):
    """
    Convenience function for plotting entire time series to view start and stop information.
    """
    ipython.magic("%matplotlib auto")
    plt.figure()
    plt.plot(series.real)
    plt.xlabel('Samples')
    plt.ylabel('Amplitude')
    plt.grid()
    plt.show()    
    
def find_delay(series1, series2):
    '''
    Accept two time series and find delay between the two via cross-correlation.
    NOTE: use only the noise portion of the time series.
    '''
    n = len(series1)
    corr = signal.correlate(series1, series2, mode='same') / np.sqrt(signal.correlate(series1, series1, mode='same')[int(n/2)] * signal.correlate(series2, series2, mode='same')[int(n/2)])
    # corr = signal.correlate(series1, series2, mode='same')
    delay = -n/2 + np.argmax(np.abs(corr))
    plt.figure()
    plt.plot(corr)
    plt.grid()
    plt.xlabel('Samples')
    plt.title('Series2 is ' + str(delay) + ' samples behind Series1')
    plt.show()
    return int(delay)

def align_data(parent_series1, parent_series2, delay):
    '''
    Align time series after finding delay. 
    NOTE: use entire time series.
    '''
    parent_series2 = np.roll(parent_series2, int(delay))
    if delay > 0:
        series1 = parent_series1[delay:]
        series2 = parent_series2[delay:]
    elif delay <= 0:
        series1 = parent_series1[:delay]
        series2 = parent_series2[:delay]
    return series1, series2

def simple_fft(series1, series2):
    """
    Take two arrays, multiply by window function and Fourier transform.
    NOTE: use after align_data()
    """
    window = blackmanharris(len(series1))
    series1_fft = fftshift(fft(window*series1))
    series2_fft = fftshift(fft(window*series2))
    return series1_fft, series2_fft

def phase_diff_1D(series1_fft, series2_fft):
    """
    Find phase difference between each bin of two FFT arrays.
    NOTE: use after simple_fft().
    """
    y1 = series1_fft.imag
    x1 = series1_fft.real
    y2 = series2_fft.imag
    x2 = series2_fft.real
    phase_difference = np.arctan2(y1*x2 - y2*x1, x1*x2 + y2*y1)
    phase_difference = phase_difference * 180/np.pi
    return phase_difference

def find_peak(series, bins):
    '''
    Accept phase difference array, take histogram, and find peaks. The distribution is Gaussian so those 
    peaks will tell me the mean/median of the phase difference, i.e. the actual phase difference.
    '''
    hist = np.histogram(series, bins=bins, density=True)
    phase_offset = hist[1][int(np.argmax(hist[0]))]
    return phase_offset

def phase_shift(angle_in_degrees):
    '''
    Returns a complex number which when multiplied with another complex number causes a 
    phase shift. Argument is in degrees by default.
    '''
    phase_shift = complex(math.cos(np.deg2rad(angle_in_degrees)), math.sin(np.deg2rad(angle_in_degrees)))
    return phase_shift

def plot_2fft(fft1, fft2, F_s):
    """
    Just a reminder how to plot FTs. Two FFTs are overplotted and their peaks are located.
    """
    N_DFT1 = len(fft1)
    N_DFT2 = len(fft2)
    freq = np.linspace(-F_s/2, F_s/2, len(fft1))
    freq_bin1 = (np.argmax(np.abs(fft1)) - N_DFT1/2)
    freq1 = freq_bin1/(N_DFT1/2)*F_s/2
    freq1 = round(freq1, 3)
    freq_bin2 = (np.argmax(np.abs(fft2)) - N_DFT2/2)
    freq2 = freq_bin2/(N_DFT2/2)*F_s/2
    freq2 = round(freq2, 3)
    
    plt.figure()
    p1, = plt.plot(freq, 20*np.log10(np.abs(fft1)/len(fft1)))
    p2, = plt.plot(freq, 20*np.log10(np.abs(fft2)/len(fft2)))
    plt.xlabel('Frequency (MHz)')
    plt.ylabel('Amplitude (dB)')
    plt.legend([p1, p2], ('FFT1', 'FFT2'))
    plt.title('FFT1 peak = ' +str(freq1) + ' MHz, FFT2 peak = ' + str(freq2) + ' MHz.')
    plt.grid()
    plt.show()

def sine_diff(s1, s2):
    dot = s1.real*s2.real + s1.imag*s2.imag
    theta = np.arccos(dot/(np.abs(s1)*np.abs(s2)))
    theta = np.degrees(theta)
    return theta

def phase_v_freq(series1, series2, N_DFT, F_s, num_tones, start_index, delay, phase_offset):
    """
    Take two time series consisting of a sequence of tones. Find phase difference for each frequency.
    Loops are initiated after every 1e6 samples.
    NOTE: do this after making phase and delay corrections.
    """
    start = start_index
    stop = start_index + N_DFT
    phase_error = []
    for n in range(num_tones):
        s1 = series1[start + n*int(1e6):stop + n*int(1e6)]
        s2 = series2[start + n*int(1e6):stop + n*int(1e6)]
        s1, s2 = align_data(s1, s2, delay)
        s2 = phase_shift(phase_offset)*s2
        s1_fft, s2_fft = simple_fft(s1, s2)
        p_error = phase_diff_1D(s1_fft[np.argmax(np.abs(s1_fft))], s2_fft[np.argmax(np.abs(s2_fft))])
        if np.argmax(np.abs(s1_fft)) == np.argmax(np.abs(s2_fft)):
            print('Bins are aligned. n = ' +str(n))
        else:
            # p_error = p_error - 180
            print("Bins are MISALIGNED. n = " +str(n))
            plot_2fft(s1_fft, s2_fft, F_s)
        phase_error.append(p_error)
    phase_error = np.array(phase_error)
    return phase_error

def phase_error_tone(series1, series2, N_DFT, F_s, start_index, delay, phase_offset):
    """
    Test to see what the phase difference is at a particular tone.
    NOTE: do this to identify errors in specific plot points after runnign phase_v_freq()
    #FIXME: something is wrong with this function.
    """
    start = start_index
    stop = start + N_DFT
    s1 = series1[start:stop]
    s2 = series2[start:stop]
    s1, s2 = align_data(s1, s2, delay)
    s2 = phase_shift(phase_offset)*s2
    s1_fft, s2_fft = simple_fft(s1, s2)
    phase_error = phase_diff_1D(s1_fft[np.argmax(np.abs(s1_fft))], s2_fft[np.argmax(np.abs(s2_fft))])
    if np.argmax(np.abs(s1_fft)) == np.argmax(np.abs(s2_fft)):
        print('Bins are aligned.')
    else:
        print("Bins are MISALIGNED.")
        # phase_error = phase_error - 180
    print(colored('Phase error is: ' +str(round(phase_error, 2)) + ' degrees.', 'magenta'))
    plt.figure()
    p1, = plt.plot(s1)
    p2, = plt.plot(s2)
    plt.xlabel('Samples')
    plt.ylabel('Amplitude')
    plt.title('Time series after correction.')
    plt.legend([p1, p2], ('Series1', 'Series2'))
    plt.grid()
    plt.show()
    plot_2fft(s1_fft, s2_fft, F_s)
    return phase_error

def FTaverage(timeDomainVector, N_DFT, averages, startIndex=0):
    """
    Calculates Fourier transform and performs averaging.
    Breaks up the time-domain data into chunks of length=N_DFT. Number of chunks=averages.
    A Fourier transform is then carried out on each of the chunks with a window (default = Blackman Harris) function.
    The FT sweeps are then running averaged and the result is one FFT chunk. The FT is normalized by N_DFT.
    """
    import scipy.fftpack
    from scipy.fftpack import fft, fftshift
    from scipy.signal import blackmanharris
    
    window = scipy.signal.blackmanharris(N_DFT) 
    j = np.arange(0, averages, 1)
    k = np.arange(0, N_DFT, 1)
    timeSignal = []
    for u in timeDomainVector[startIndex:startIndex+averages*N_DFT]:
        timeSignal.append(u)
    freqSignal = []
    for v in j:
        FFTsweep = scipy.fftpack.fftshift(scipy.fft(window*timeSignal[int(N_DFT*v):int(N_DFT+v*N_DFT)]))/N_DFT
        freqSignal.append(FFTsweep)
    freqSignal = np.array(freqSignal)
    avged_FT = []
    for w in k:
        avged_bin = np.sum(np.abs((freqSignal[:, w])))
        avged_FT.append(avged_bin)
    avged_FT = np.array(avged_FT)
    avged_FT = avged_FT/averages
    return avged_FT

def take_fft_sweep(series1, series2, N_DFT):
    """
    Take time series and break down into 2D array of FFT sweeps
    NOTE: Use the noise data from the parent series after running align_data().
    """
    window = scipy.signal.blackmanharris(N_DFT)
    series1_fft = []
    series2_fft = []
    if len(series1) == len(series2):
        averages =int(np.floor(len(series1)/N_DFT))
    else:
        raise Exception("Time series 1 and 2 need to be of equal length.")
    for m in range(averages):
        series1_trial = series1[m*N_DFT:N_DFT*(m+1)]
        series1_trial = fftshift(fft(window*series1_trial))
        series1_fft.append(series1_trial)
        series2_trial = series2[m*N_DFT:N_DFT*(m+1)]
        series2_trial = fftshift(fft(window*series2_trial))
        series2_fft.append(series2_trial)
    series1_fft = np.array(series1_fft)
    series2_fft = np.array(series2_fft)
    return series1_fft, series2_fft

def avg_fft(series_fft):
    """
    Take a 2D array of several FFT sweeps and return average of complex values,
    i.e. coherent average.
    NOTE: perform after take_fft_sweep()
    """
    fft_avg = []
    for m in range(series_fft.shape[1]):
        fft_avg.append((np.sum(series_fft[:, m])/series_fft.shape[0]))
    return np.array(fft_avg)    
    
def phase_diff_2D(series1_fft, series2_fft):
    """
    Take two 2-D arrays containing FTs of broken down time series data, and find the phase
    angle difference between each corresponding bin.
    NOTE: use output of take_fft_sweep().
    """
    phase_diff_vector = []
    for m in range(series1_fft.shape[0]):
        # phase_difference = np.angle((series1_fft[m]), deg=True) - np.angle((series2_fft[m]), deg=True)
        phase_difference = phase_diff_1D(series1_fft[m], series2_fft[m])
        phase_diff_vector.append(phase_difference)
    phase_diff_vector = np.array(phase_diff_vector)
    return phase_diff_vector

def phase_avg(phase_diff):
    """
    Take vector containing the phase differences between bins of two FTs and find averaged
    phase difference.
    NOTE: use output of phase_diff().
    """
    phase_avg_vector = []
    for m in range(phase_diff.shape[1]):
        phase_average = np.sum(phase_diff[:, m])
        phase_avg_vector.append(phase_average)
    phase_avg_vector = np.array(phase_avg_vector)
    phase_avg_vector = phase_avg_vector/phase_diff.shape[0]
    return phase_avg_vector

def hist_avg(phase_diff_array, bins):
    """
    Take 2D array of phase differences between FFT bins and find averaging by weighting 
    histogram bins.
    NOTE: use 2D array derived from phase_diff().
    """
    hist_vector = []
    for m in range(phase_diff_array.shape[0]):
        hist = np.histogram(phase_diff_array[m], bins=bins, density=True)
        hist_vector.append(hist[0])     # get weights only
    hist_vector = np.array(hist_vector)
    hist_avg_vector = []
    for n in range(bins):
        hist_avg = np.sum(hist_vector[:, n])
        hist_avg_vector.append(hist_avg)
    hist_avg_vector = np.array(hist_avg_vector)
    hist_avg_vector = hist_avg_vector/phase_diff_array.shape[1]
    return hist_vector, hist_avg_vector

def phase_curve(series1, series2, N_DFT, averages):
    """
    Find phase difference after averaging several FFTs.
    NOTE: do after align_data()
    """
    for m in range(averages):
        fft1 = fftshift(fft(series1[N_DFT*m:N_DFT*(m+1)]))
        fft2 = fftshift(fft(series2[N_DFT*m:N_DFT*(m+1)]))
        phase = phase_diff_1D(fft1, fft2)
        return phase
    
def angle(x,y):

    dot = x.real*y.real + x.imag*y.imag
    det = x.real*y.imag - x.imag*y.real
    return np.arctan2(det, dot)

def find_residual_phase(a,b,window_size=1024):

    n_windows =int(np.floor(len(a)/window_size))

    phase = np.zeros(window_size)
    for i in range(n_windows):
        low = window_size*i
        high = window_size*(i+1)
        fft_a = fftshift(fft(a[low:high]))
        fft_b = fftshift(fft(b[low:high]))
        phase += angle(fft_a,fft_b)

    phase/=n_windows
    print(n_windows)
    return phase