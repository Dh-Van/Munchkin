import pickle
import numpy as np

with open('recordings/Scoop_NEW.pkl', 'rb') as f:
    angles = pickle.load(f)
    
    window_size = 20
    smoothed_angles = []
    
    raw_data = np.array(angles)
    
    for i in range(raw_data.shape[1]):
        smooth_col = np.convolve(raw_data[:, i], np.ones(window_size)/window_size, mode='valid')
        smoothed_angles.append(smooth_col)
        
    final_data = np.array(smoothed_angles).T.tolist()

    with open('recordings/Scoop_TEST.pkl', 'wb') as f:
        pickle.dump(final_data, f)
