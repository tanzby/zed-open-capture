import os
import csv
import rosbag
import argparse
import cv2
import numpy as np
import shutil

##################
# DESCRIPTION:
# Creates CSV files of the sensor_msgs::Imu from a rosbag
#
# USAGE EXAMPLE:
# python export_zed_data.py --bag your_bag.bag --save /path/to/save/path
###################

""" Read parameters """
parser = argparse.ArgumentParser(description='Extract RosBag message.')
parser.add_argument('-b', '--bag', type=str, help='bag file you want to extract', required=True)
parser.add_argument('-s', '--save', type=str, help='path data will extract to', required=True)
parser.add_argument('-i', '--imu_topic', type=str, default='/zed_node/imu/data')
parser.add_argument('-l', '--left_topic', type=str, default='/zed_node/left/image_raw/compressed')
parser.add_argument('-r', '--right_topic', type=str, default='/zed_node/right/image_raw/compressed')
args = parser.parse_args()

""" Read bag file """
filename = args.bag
directory = args.save

imu_topic = args.imu_topic
left_topic = args.left_topic
right_topic = args.right_topic

print("Reading the rosbag file")
if not directory.endswith("/"):
    directory += "/"
extension = ""
if not filename.endswith(".bag"):
    extension = ".bag"
bag = rosbag.Bag(directory + filename + extension)

""" Create directory with name filename (without extension) """
results_dir = directory + filename[:-4] + "_results"
if not os.path.exists(results_dir):
    os.makedirs(results_dir)
else:
    shutil.rmtree(results_dir)
    os.makedirs(results_dir)

left_image_dir = results_dir + '/left/'
if not os.path.exists(left_image_dir):
    os.makedirs(left_image_dir)

right_image_dir = results_dir + '/right/'
if not os.path.exists(right_image_dir):
    os.makedirs(right_image_dir)

print("Writing sensor_msgs::Imu data to CSV, Extract Image to png")

imu_data_file = open(results_dir + "/" + filename + '_imu_data.csv', mode='w')
left_timestamp_data_file = open(results_dir + "/" + filename + '_left_timestamp.csv', mode='w')
right_timestamp_data_file = open(results_dir + "/" + filename + '_right_timestamp.csv', mode='w')

imu_data_writer = csv.writer(imu_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
imu_data_writer.writerow(['# timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

left_timestamp_writer = csv.writer(left_timestamp_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
left_timestamp_writer.writerow(['# timestamp'])

right_timestamp_writer = csv.writer(right_timestamp_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
right_timestamp_writer.writerow(['# timestamp'])


def convert_img(msg):
    data = np.fromstring(msg.data, np.uint8)
    data = cv2.imdecode(data, cv2.IMREAD_COLOR)
    return data


# Get all message on the sensor_msgs::Imu topic
for topic, msg, t in bag.read_messages(topics=[imu_topic, right_topic, left_topic]):
    # Only write to CSV if the message is for our robot
    if topic == imu_topic:
        a = msg.angular_velocity
        g = msg.linear_acceleration
        imu_data_writer.writerow([msg.header.stamp.to_nsec(), a.x, a.y, a.z, g.x, g.y, g.z])
    elif topic == left_topic:
        img_data = convert_img(msg)
        cv2.imwrite("%s%d.png" % (left_image_dir, msg.header.stamp.to_nsec()), img_data)
        left_timestamp_writer.writerow([t])
    elif topic == right_topic:
        img_data = convert_img(msg)
        cv2.imwrite("%s%d.png" % (right_image_dir, msg.header.stamp.to_nsec()), img_data)
        right_timestamp_writer.writerow([t])

print("Finished extracting data!")
bag.close()
