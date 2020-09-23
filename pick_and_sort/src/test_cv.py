#!/usr/bin/env python

# import numpy as np
# import cv2
import boto3
import json
import numpy as np
import re

# file_name = '/home/mohamed/Desktop/asd/red_1.jpg'
file_name = "/home/mohamed/Desktop/ooo.jpg"
endpoint_name = 'DEMO-imageclassification-ep--2020-08-19-06-17-16'
session = boto3.Session(
    aws_access_key_id="AKIAIQ3XPMYKF6OOI2VQ",
    aws_secret_access_key="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",
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