import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
from scipy.interpolate import griddata

class CameraModel():
    def __init__(self, dataset):
        self.dataset = dataset

    def convertLidar(self, scan):
        # create copy
        newScan = scan.copy()
        newScan[:, 3] = 1

        # create copy of scans to add to original after matrix modification occurs
        addScan = np.delete(scan.copy(), 3, axis=1)

        # convert xy points from lidar to camera coordinates
        newScan = np.dot(self.dataset.calib.T_cam2_velo, newScan.T)
        newScan = np.dot(self.dataset.calib.P_rect_20, newScan)
        newScan = newScan.T
        newScan = np.hstack((newScan, addScan))
        negativeMask = newScan[:, 2] > 0
        newScan = newScan[negativeMask]

        # normalize depth values
        newScan[:, 0] = newScan[:, 0] / newScan[:, 2]
        newScan[:, 1] = newScan[:, 1] / newScan[:, 2]
        newScan[:, 2] = np.log(newScan[:, 2])

        # filter points to fit on camera images
        boundsMaskX = (newScan[:, 0] < 1241) & (newScan[:, 0] > 0)
        newScan = newScan[boundsMaskX]
        boundsMaskY = (newScan[:, 1] < 374) & (newScan[:, 1] > 0)
        newScan = newScan[boundsMaskY]

        return newScan

    def videoGen(self, choice):
        #create generators and objects
        genRGB = self.dataset.rgb
        genVelo = self.dataset.velo
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video = cv2.VideoWriter('fullDrive.mp4', fourcc, 10, (1242, 375))
        startTime = time.perf_counter()

        for img, scan in zip(genRGB, genVelo):

            newScan = self.convertLidar(scan)

            #draw image and scan data on top
            cam2, cam3 = img
            fig, ax = plt.subplots(figsize=(12.42, 3.75), dpi=100)
            fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
            ax.set_position([0, 0, 1, 1])
            ax.imshow(cam2)
            if choice == 1:
                scatter = ax.scatter(newScan[:, 0], newScan[:, 1], s=1, c=newScan[:, 2], cmap='viridis')
            elif choice == 2:
                n1, n2, d = self.depthInterpolate(newScan)
                scatter = ax.scatter(n1.ravel(), n2.ravel(), s=0.03, c=d.ravel(), cmap='viridis')
            ax.axis('off')
            fig.canvas.draw()

            #convert plt figure to array for cv2 to process and make video
            frame = np.asarray(fig.canvas.buffer_rgba())
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            video.write(frame)

            plt.close()

        video.release()
        cv2.destroyAllWindows()
        endTime = time.perf_counter()
        print(f"Program runtime: {endTime - startTime:.2f} seconds")

    def depthInterpolate(self, scan):
        a = np.arange(1242)
        b = np.arange(375)
        c1, c2 = np.meshgrid(a, b)
        xCoord = scan[:, 0]
        yCoord = scan[:, 1]
        depthMap = griddata(np.column_stack((xCoord, yCoord)), scan[:, 2], (c1, c2), method='nearest')
        c1 = c1[141:]
        c2 = c2[141:]
        depthMap = depthMap[141:]
        depthMap = c1, c2, depthMap

        return depthMap