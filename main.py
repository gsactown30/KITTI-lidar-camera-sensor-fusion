import pykitti
from camera_model import CameraModel

# Load the dataset
basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)

cm = CameraModel(dataset)

choice = int(input('regular(1) or depth interpolated lidar scan(2)'))
cm.videoGen(choice)