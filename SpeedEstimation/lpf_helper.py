def butter_lowpass_filter(data, cutoff, fs, order=4):
    """
    Zero-phase Butterworth Low-Pass Filter.
    data: (T,) or (T, D)
    cutoff: Cutoff frequency (Hz)
    fs: Sampling frequency (Hz)
    order: Filter order
    """
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    
    if data.ndim == 1:
        y = filtfilt(b, a, data)
    else:
        # data is (T, D), apply along time axis 0
        y = filtfilt(b, a, data, axis=0)
    return y
