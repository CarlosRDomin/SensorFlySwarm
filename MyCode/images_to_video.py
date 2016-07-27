import cv2
import os
from datetime import datetime


def generate_video_from_images(img_folder_or_filelist, video_filename="", video_format="mp4", video_fps=15):
	t_start = datetime.now()

	if isinstance(img_folder_or_filelist, list):
		files = img_folder_or_filelist

		if video_filename == "":  # When img_folder_or_filelist is a list of files, can't deduct video filename from it
			video_filename = "video"  # So just make sure it's not empty
	elif isinstance(img_folder_or_filelist, str):
		files = list()
		files_in_folder = os.listdir(img_folder_or_filelist)
		for f in files_in_folder:
			f_full_path = os.path.join(img_folder_or_filelist, f)  # Need to append folder path in front of file name
			if os.path.isfile(f_full_path):
				files.append(f_full_path)

		if video_filename == "":
			video_filename = os.path.split(img_folder_or_filelist)[-1]  # Name the video file after the parent ("deepest") folder
	else:
		raise TypeError("img_folder_or_filelist can either be a list of images, or a folder (str) where all images are")

	if len(files) > 0:
		if video_filename[-(1+len(video_format)):] != ".{}".format(video_format):  # If filename doesn't end in the right file extension
			video_filename = "{}.{}".format(video_filename, video_format)  # Manually add it
		first_frame = cv2.imread(files[0])
		video = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'X264'), video_fps, (first_frame.shape[1], first_frame.shape[0]))

		for i, f in enumerate(files):
			video.write(cv2.imread(f))
			print "\r{} out of {} frames ({:6.2f}%) written! Time elapsed: {:.2f}s".format(i+1, len(files), 100.0*(i+1)/len(files), (datetime.now()-t_start).total_seconds()),

		video.release()  # Make sure to release the video so it's actually written to disk
		print "\nVideo successfully saved as '{}' in a total of {:.2f}s! :)".format(video_filename, (datetime.now()-t_start).total_seconds())


if __name__ == '__main__':
	generate_video_from_images("img/2016-06-22 13:45:07", video_fps=25)
