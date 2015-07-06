# -*- coding: utf-8 -*-

HOME_DIR = r"D:\\Documents\\Data\\MSL\\FRBG3_1\\rgbd_dataset_freiburg3_long_office_household\\"

RGB_FILE_PATH = HOME_DIR + "rgb\\"                            #Path where the RGB files are stored
DEPTH_FILE_PATH = HOME_DIR + "depth\\"                        #Path where the depth maps are stored
_2D_FEATURE_DUMP_PATH = HOME_DIR + "Output.csv"               #Path to dump feature vectors

GROUND_TRUTH_FILE = HOME_DIR + "groundtruth.txt" #Ground truth file for the camera position, as given in the computer vision dataset from Technische Universität München
""" Check: http://vision.in.tum.de/data/datasets/rgbd-dataset/download# 
    for more details  """
""" For the first iteration, the camera position is being read from a ground truth file (which has been obtained by externally tracking the camera).
    This saves us the necessity of implementing the Extended Kalman Filter at the very onset.
    We use this as an oppurtunity to build and debug our test environment, as well as coding the module for initializing depth vectors from features.
    Please check documentation for "Iteration-1", for more details regarding this approach"""    
    
DEPTH_IMAGE_LIST = HOME_DIR + "depth.txt"        #Location to the text file containing list of all depth images
RGB_IMAGE_LIST = HOME_DIR + "rgb.txt"            #Location of the text file containing list of all RGB images

ARRAY_DUMP_PATH = HOME_DIR
ARRAY_DUMP_PREFIX = "FeatureArray"

"""parameters for associate.py"""
ASSOCIATE_OFFSET = 0.0              #time offset added to the timestamps of the second file (default: 0.0)
ASSOCIATE_MAX_DIFFERENCE = 0.02     #maximally allowed time difference for matching entries(default: 0.02)
DEPTH_FILE_TYPE = ".png"
FILE_NAME_LENGTH = 17
RGB_DEPTH_MATCH_THRESHOLD = 0.000001
"""+++++++++++++++++++++++++++"""

DEBUG = 0                                       # enable/disable debug dumps. 1 turns it on, 0 switches it off
VR = 1                                          # Enable/disable visual representation OR graphs
LOG_FILE = 1                                    # Toggle writing of logs to file
IS_INVERSE_DEPTH_PARAMETRIZATION = 0            # assign 1 to this variavle if using Inverse Depth Parameterization, assign 0 is using XYZ representation
IS_LOCALIZATION_FROM_GT = 1                     # assign 1 to indicate that the current position of camera is being read from Groundtruth file (and not using SFM/homography estimation)
GT_DATA_SIZE = 100                               # Number of entries that we are using from position GT file
TOTAL_GT_DATA_SIZE = 8710                       # Total number of entries in GT File, required for finding positional GT for RGB images

"""debugging parameters for OpticalFlow.py"""

SHOW_FEATURES_2D = 1                            # Show the image containing 2D images along with the detected features (marked as circles) in a window. [Feature detection and tracking using OpenCV stuff]
SAVE_FEATURES_2D_IMG = 1                        # Save the image containing 2D images along with the detected features (marked as circles)
FEATURES_2D_SAVE_DIR = HOME_DIR + "features\\"  # Path to save the images, if SAVE_FEATURES_2D_IMG is enabled
FEATURE_IMG_NAME = "features"
FEATURE_IMG_EXTN = ".png"
FEATURE_MATCHING_FILE = FEATURES_2D_SAVE_DIR + "features.txt"

""" +++++++++++++++++++++++++++++++++++++ """


IMAGE_READ_OFFSET = 500

GT_START_INDEX = 0                              # Line number of ground truth file at which we should start reading the position from
GT_END_INDEX = GT_DATA_SIZE                     # Line number of ground truth file at which we should end reading the position
GT_HEADER_END_LINE = 3                          # Line number at which the ground truth file's header information ends

PARTICLE_FILTER = 1                             #Enables feature depth mapping using particle filtering
PARTICLE_INIT_DEPTH = 0.5                       # Depth at which the depth-particles would be initiated from
PARTICLE_COUNT = 100                            # Number of depth-particles
PARTICLE_INTERVAL = 0.045                       # Seperation at which the depth-particles are intialized
""" Check https://www.doc.ic.ac.uk/~ajd/Publications/civera_etal_tro2008.pdf for more details on Inverse Depth Parametrization for Monocular SLAM """


FEATURE_SIZE = 10                                   # Number of features to be extracted from each frame/image
MAX_OBSERVATION = GT_END_INDEX - GT_START_INDEX + 1 # Maximum number of frames/images/observations allowed in a single session
MAX_FEATURES = FEATURE_SIZE * MAX_OBSERVATION       # Maximum possible number of feature points extracted in one session
SCALED_OBSERVATION_SIZE = 199

MIN_FEATURE_DISTANCE = 4                        # Minimum Eucledian distance between two consecutive feature positions on image plane (pixel coordinates), as obtained from Shi-Tomasi feature detector

WORLD_SCALE_X = 100                            # Scale of the entire map along X - axis
WORLD_SCALE_Y = 100                            # Scale of the entire map along Y - axis
WORLD_SCALE_Z = 100                            # Scale of the entire map along Z - axis


""" ****CAMERA STATE SPACE REPRESENTATION**** """
POSITION_VECTOR_SIZE = 3                        # Defining the size of the vector representing the position of the camera
QUATERNION_SIZE = 4                             # Defining the size of the unit quaternion representing the current orientation of the camera
TRANSLATIONAL_VELOCITY_VECTOR_SIZE = 3          # Defining the size of the vector representing the instantenous translational velocity of the camera
ANGULAR_VELOCITY_VECTOR_SIZE = 3                # Defining the size of the vector representing the instantenous angular velocity of the camera


""" ********INITIALIZATION PARAMETERS********  """

INITIAL_DEPTH_ESTIMATE = 0.5

INIT_X = 0
INIT_Y = 0
INIT_Z = 0

QUAT_INIT_REAL = 1
QUAT_INIT_I = 0
QUAT_INIT_J = 0
QUAT_INIT_K = 0

INIT_V_X = 0
INIT_V_Y = 0
INIT_V_Z = 0

INIT_OMEGA_X = 0
INIT_OMEGA_Y = 0
INIT_OMEGA_Z = 0




""" **********CAMERA INTRINSIC PARAMETER************ """
""" Intrinsic parameter for RGB-D SLAM dataset of Technische Universität München, camera: Freiburg 1"""
""" For additional details, please refer: http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect"""
"""
#   ***Parameters for FRBG 1***
Fu = 517.3
Fv = 516.5
Cu = 318.6
Cv = 255.3

d0 = 0.2624
d1 = -0.9531
d2 = -0.0054
d3 = 0.0026
d4 = 1.1633
"""

#   ***Parameters for FRBG 3***
Fu = 535.4
Fv = 539.2
Cu = 320.1
Cv = 247.6

d0 = 0
d1 = 0
d2 = 0
d3 = 0
d4 = 0


# -*- coding: utf-8 -*-

