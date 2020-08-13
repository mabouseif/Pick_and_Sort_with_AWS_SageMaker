import argparse
import glob, time, pickle
from PIL import Image
import os, sys
import numpy as np
import cv2
from skimage.transform import rescale, resize, downscale_local_mean, rotate


def transform_img(img, scale, angle_deg):
    
    angle_rad = angle_deg * np.pi/180.0
    
    new_height = int(img.shape[0] * scale)
    new_width = int(img.shape[1] * scale)
    
    img_resized = cv2.resize(img, dsize=(new_width, new_height), interpolation=cv2.INTER_CUBIC)
    point_rotate_about = np.array([img_resized.shape[1]/2.0, img_resized.shape[0]/2.0]) # x, y point
    
    img_rotated = rotate(img_resized, angle_deg, resize=True, center=None, order=1, mode='constant', cval=0, clip=True, preserve_range=True).astype(np.uint8)
    shift_by = np.array([(img_rotated.shape[1]-img_resized.shape[1])/2.0, (img_rotated.shape[0]-img_resized.shape[0])/2.0]) # shift in y, shift in x
    
    return img_rotated

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--dir_jpeg', required=True, help='Directory of dataset in JPEG format')
    parser.add_argument('--dir_out', required=True, help='Directory of output augmented dataset')
    parser.add_argument('--gray', dest='gray', action='store_true', help='Convert images to gray scale')
    parser.add_argument('--scale', dest='scale', default=1.0, type=float, action='store', help='Scaling transform of the augmentations')
    parser.add_argument('--degree_increments', dest='degree_increments', default=360, type=int, action='store', help='If provided Rotate images in increments of degree_increments till 360, \
                                                                                                                      otherwise equates to covnerting images from JSON to png')

    args = parser.parse_args()

    dir_jpeg = args.dir_jpeg
    dir_out = args.dir_out
    gray = args.gray
    scale = args.scale
    degree_increments = args.degree_increments



    if not os.path.isdir(dir_jpeg):
        print("Directory invalid!")
        exit()
    
    if not os.path.exists(dir_out):
        os.mkdir(dir_out)
        print("Output directory does not exist! Created one at {}".format(dir_out))

    count = 0
    start_time = time.time()
    for file_path in glob.glob(os.path.join(dir_jpeg, '*')):
        img =  cv2.imread(file_path, cv2.IMREAD_GRAYSCALE) if gray else cv2.imread(file_path, cv2.IMREAD_COLOR)
        if img is not None:
            print('\rImage number: {}'.format(count+1), end="")
            sys.stdout.flush()
            
            for angle_deg in range(0, 360, degree_increments):
                file_dir, file_name = os.path.split(file_path)
                new_file_path = os.path.join(dir_out, file_name.split(".")[0] + '_' + str(angle_deg) + '.' + file_name.split(".")[1])
                image_transformed = transform_img(img, scale, angle_deg)
                new_height = int(image_transformed.shape[0] * scale)
                new_width = int(image_transformed.shape[1] * scale)
                cv2.imwrite(new_file_path, image_transformed)
        count += 1
        
    end_time = np.round((time.time() - start_time) / 60.0, 2)
    print('\nElapsed: {} minutes'.format(end_time))