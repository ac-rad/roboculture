import cv2
import numpy as np 
import csv
import time
from matplotlib import pyplot as plt
import os

class ProcessWellImage:

    def __init__(self):
        self.init_time = time.time()
        self.experiment_time = time.strftime("%Y-%h-%d-%H:%M:%S")

    def process_well_image(self, image, well_num, iteration):
        print(f'Processing image for well {well_num}')

        # open csv file
        # if csv file does not exist, create it and write header
        if not os.path.exists(f'src/cell_culture/well_growth_{self.experiment_time}'):
            os.makedirs(f'src/cell_culture/well_growth_{self.experiment_time}')
        
        if not os.path.exists(f'src/cell_culture/well_growth_{self.experiment_time}/well_growth.csv'):
            with open(f'src/cell_culture/well_growth_{self.experiment_time}/well_growth.csv', 'w') as f:
                f.write('time, iteration, well_num, avg_h, avg_s, avg_v\n')
        with open(f'src/cell_culture/well_growth_{self.experiment_time}/well_growth.csv', 'a') as f:
            # if empty, write header
            if f.tell() == 0:
                f.write('time, iteration, well_num, avg_h, avg_s, avg_v\n')

            if not os.path.exists(f'src/cell_culture/well_growth_{self.experiment_time}/{iteration}'):
                os.makedirs(f'src/cell_culture/well_growth_{self.experiment_time}/{iteration}')
            
            cv2.imwrite(f'src/cell_culture/well_growth_{self.experiment_time}/{iteration}/{well_num}.png', image)

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # get average hsv values
            avg_h = np.mean(hsv[:,:,0])
            avg_s = np.mean(hsv[:,:,1])
            avg_v = np.mean(hsv[:,:,2])

            # write to csv
            # f.write(f'{time.time() - self.init_time}, {iteration}, {well_num}, {avg_h}, {avg_s}, {avg_v}\n')
            f.write(f'{time.time()}, {iteration}, {well_num}, {avg_h}, {avg_s}, {avg_v}\n')

        return avg_v
