#!/usr/bin/env python


"""AWS Sagemaker Cloud Inference Script

This script allows the user to query for cloud inference on AWS
Sagemaker for images captured from ROS and Gazebo

The script reads a (.jpg) image file that was converted from ROS
image format, uploads it to the AWS Sagemaker endpoint for
inference, decodes the result, and prints the result.

The accepted file format is (.jpg).

The script requrest Boto, the Amazon Web Services (AWS) SDK for Python,
to be installed, as well as numpy. Remaining required modules are 
built-in.

The script is called from the ROS C++ interface.

"""


# import cv2
import boto3
import json
import numpy as np
import re

# file_name = '/home/mohamed/Desktop/asd/red_1.jpg'
file_name = "/home/mohamed/Desktop/object_image.jpg"
endpoint_name ='endpoint-new'
session = boto3.Session(
    aws_access_key_id="xx",
    aws_secret_access_key="xx",
    # aws_session_token=SESSION_TOKEN
)
runtime = session.client(service_name='sagemaker-runtime',region_name='eu-west-1')

with open(file_name, 'rb') as f:
        payload = f.read()
        payload = bytearray(payload)
response = runtime.invoke_endpoint(EndpointName=endpoint_name, ContentType='application/x-image', Body=payload)
# print(response['Body'].read())
# print(response['Body'].read().decode('ascii').split())
a, b = (response['Body'].read().decode('ascii').split())
a = int(round(float(a[1:-1])))
b = int(round(float(b[0:-1])))
result = [a, b]
# print(result)
index = np.argmax(result)
object_categories = ["red_box", "blue_box"]

# print(str(result[index]))
print(str(object_categories[index]))

# print("Result: label - " + object_categories[index] + ", probability - " + str(result[index]))