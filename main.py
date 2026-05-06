import pykitti
from camera_model import CameraModel

# Load the dataset
basedir = './data/kitti'
date = '2011_09_26'
drive = '0015'

dataset = pykitti.raw(basedir, date, drive)

cm = CameraModel(dataset)

cm.videoGen()