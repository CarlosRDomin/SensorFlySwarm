"""
	Simple script that combines all individual images corresponding to the "visual log" of an experiment
	and generates a video with them, so it's easier to visualize what happened.
"""

import cv2
import os
from datetime import datetime


def generate_video_from_images(img_folder_or_filelist, video_folder="videos", video_filename="", video_format="mp4", video_fps=15, img_filelist_prefix=""):
	"""
	Core function to combine a bunch of images into a single video file.
	:param img_folder_or_filelist: Either a list of all images to be used or the path to a folder containing all the images
	:param video_folder: Path to the folder where the output video is to be stored
	:param video_filename: Name of the actual video file. Leave empty ("") to rename it after the experiment date (parent folder of the images, if img_folder_or_filelist is a string)
	:param video_format: String representing the desired format (extension) for the output video. Eg: mp4
	:param video_fps: Output video frame rate
	:param img_filelist_prefix: In case of img_folder_or_filelist being a 'filelist' (str), indicates what
	prefix should the files in that folder start with in order to be added to the video (eg: "out_" for output frames)
	"""
	t_start = datetime.now()

	if isinstance(img_folder_or_filelist, list):
		files = img_folder_or_filelist

		if video_filename == "":  # When img_folder_or_filelist is a list of files, can't deduct video filename from it
			video_filename = "video"  # So just make sure it's not empty
	elif isinstance(img_folder_or_filelist, str):
		files = list()
		files_in_folder = os.listdir(img_folder_or_filelist)
		for f in files_in_folder:
			if not f.startswith(img_filelist_prefix): continue  # Only consider files that start with img_filelist_prefix
			f_full_path = os.path.join(img_folder_or_filelist, f)  # Need to append folder path in front of file name
			if os.path.isfile(f_full_path):
				files.append(f_full_path)

		if video_filename == "":
			video_filename = os.path.split(img_folder_or_filelist)[-1]  # Name the video file after the parent ("deepest") folder
	else:
		raise TypeError("img_folder_or_filelist can either be a list of images, or a folder (str) where all images are")

	if len(files) > 0:
		if not os.path.exists(video_folder):  # Make sure video_folder exists, otherwise create it
			os.makedirs(video_folder)

		if video_filename[-(1+len(video_format)):] != ".{}".format(video_format):  # If filename doesn't end in the right file extension
			video_filename = "{}.{}".format(video_filename, video_format)  # Manually add it
		video_filename = os.path.join(video_folder, video_filename)  # Construct the whole filename by combining the folder path and the actual name of the file
		first_frame = cv2.imread(files[0])  # Read the first frame to get its frame size
		video = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'X264'), video_fps, (first_frame.shape[1], first_frame.shape[0]))

		for i, f in enumerate(files):
			video.write(cv2.imread(f))
			print "{} out of {} frames ({:6.2f}%) written! Time elapsed: {:.2f}s".format(i+1, len(files), 100.0*(i+1)/len(files), (datetime.now()-t_start).total_seconds()),

		video.release()  # Make sure to release the video so it's actually written to disk
		print "Video successfully saved as '{}' in a total of {:.2f}s! :)".format(video_filename, (datetime.now()-t_start).total_seconds())


if __name__ == '__main__':
	generate_video_from_images(os.path.join("img-ns", "2016-11-03 03-05-11"), video_fps=20, img_filelist_prefix="out_")
