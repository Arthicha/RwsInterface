# system module
import os, sys, inspect
import csv
os.chdir(os.path.dirname(os.path.abspath(__file__)))
# remove warning
import warnings
warnings.filterwarnings("ignore")
# pytorch (with cuda)
import torch
import torch.nn as nn
import torch.nn.functional as F
# opencv & numpy
import cv2
import numpy as np
import copy
from numpy import linalg as LA
import matplotlib.pyplot as plt 
# feature extraction models
import feature_ext_models.CARN.carn as carn


MODEL = "CARN"

if len(sys.argv) != 9:
	print("ERROR: input arguments do not match!")
	sys.exit()

method = int(sys.argv[1]) # feature (RGB vs CARN)
f = float(sys.argv[2]) # focal length
tx = float(sys.argv[3]) # distance between two cameras
window = float(sys.argv[4]) # sliding window
stride = float(sys.argv[5]) # sliding stride
padding = float(sys.argv[6]) # padding
nfeature = float(sys.argv[7]) # number of strong match
noisevar = 0.00000001*float(sys.argv[8]) # image noise

def Transform(im):
	# numpy/cv2 image -> torch image
	im = np.uint8(im)
	im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
	im = im[180:320,:,:]
	im = torch.FloatTensor(im).cuda()/255
	im = im.unsqueeze(0).permute(0,3,1,2)
	return im

def deTransform(im,mx=255):
	# torch image -> numpy/cv2 image
	im = im.permute(0,2,3,1).squeeze(0)
	im = torch.clamp(im,0,1)*mx
	im = im.type(torch.uint8).cpu().numpy()
	img = np.zeros((480,640,im.shape[2]),dtype=np.uint8)
	img[180:320,:,:] = im
	return img

def getMask(img,null):
	# calculate mask from image substraction
	mask = img*0
	mask[100:420,:,:] = np.array([255,255,255])
	diff = LA.norm(img-null,axis=2)
	mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
	mask = np.where(diff < 10,0.0,255.0)
	mask[:120,:] = 0
	return mask



def Sort(sub_li,elem_id):
	# reverse = None (Sorts in Ascending order)
	# key is set to sort using second element of 
	# sublist lambda has been used
	return(sorted(sub_li, key = lambda x: x[elem_id])) 


def add_gaussian_noise(X_img,var):
	gaussian_noise_imgs = []
	row, col, _ = X_img.shape
	# Gaussian distribution parameters
	mean = 0
	sigma = var ** 0.5
	gaussian = np.random.normal(mean, var, (row,col,1)).astype(np.float32)
	gaussian = np.concatenate((gaussian, gaussian, gaussian), axis = 2)
	gaussian_img = np.clip(X_img.astype(np.float32)/255+gaussian,0,1.0)
	gaussian_noise_imgs = np.array(gaussian_img*255, dtype = np.uint8)
	return gaussian_noise_imgs

def ready(name, disturb=False):
	global noisevar
	imgx = cv2.imread(name)
	cv2.imwrite("step/step1_"+name,imgx)
	if disturb:
		imgx = add_gaussian_noise(imgx,noisevar)
	imgx = cv2.blur(imgx, (10,10)) 
	cv2.imwrite("step/step2_"+name,imgx)
	return imgx

# data preparation
org = [cv2.flip(ready("img"+str(i)+".png",1), 1) for i in range(2)]
null = [cv2.flip(ready("null"+str(i)+".png",0), 1) for i in range(2)]
mask = [getMask(org[i],null[i]) for i in range(2)]
for i in range(2):
	cv2.imwrite("step/step3_mask"+str(i)+".png",mask[i])
img = [org[i] * np.stack([mask[i],mask[i],mask[i]],axis=2)/255 for i in range(2)] #if stride > 1.1 else org
for i in range(2):
	cv2.imwrite("step/step4_feature"+str(i)+".png",img[i])
if 0:#stride < 1.1:
	for i in range(2):
		img[i] = cv2.blur(img[i], (10,10)) 
for i in range(2):
	cv2.imwrite("step/step5_blur"+str(i)+".png",img[i])


def getBoxes(mask,i=0):
	global img
	# calculate the bounding box
	thresh = np.uint8(mask)
	cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	best_cnts = None
	best_area = -10
	for c in cnts:
		 area = cv2.contourArea(c)
		 if area > best_area:
		 	best_cnts = c
		 	best_area = area
	cnts = np.array(best_cnts)
	imgx = copy.deepcopy(img[i])
	cv2.drawContours(imgx, [cnts], -1, (0, 0, 255), 3)
	cv2.imwrite("step/step6_contour"+str(i)+".png",imgx)
	boxes = []
	for c in [cnts]:
		x,y,w,h = cv2.boundingRect(c)
		boxes.append([x,y,w,h])
	imgx = copy.deepcopy(img[i])
	imgx = cv2.rectangle(imgx, (int(x),int(y)), (int(x+w),int(y+h)), (0,0,255),3)
	cv2.imwrite("step/step7_box"+str(i)+".png",imgx)
	return boxes

boxes = [getBoxes(mask[i],i) for i in range(2)]
diff1 = abs(boxes[0][0][0]-320)
diff2 = abs(boxes[1][0][0]-320)

# feature extraction
if method:
	model = carn.Net().cuda()
	model.load("feature_ext_models/CARN/carn.pth")
	feature = [deTransform(model(Transform(img[i]))) for i in range(2)]
	feature = [cv2.resize(feature[i],(640,480)) for i in range(2)]
else:
	feature = [img[i] for i in range(2)]

# create csv file for communication between python and c++
file = open('python_output.csv', 'w', newline='')
writer = csv.writer(file)

# preparation
matches = []
if (diff1 < diff2):
	box = boxes[0][0]
	main_img = feature[0]
	search_img = feature[1]
	rotation_mask = mask[0]
	rotation_img = org[0]
else:
	box = boxes[1][0]
	main_img = feature[1]
	search_img = feature[0]
	rotation_mask = mask[1]
	rotation_img = org[1]
dx = int(box[2]*window)-1 if window != 0.0 else 1
dy = int(box[3]*window)-1 if window != 0.0 else 1



for x in range(box[0]-int(padding*dx),box[0]+box[2]+int(padding*dx),int(stride*(box[2]+2*padding*dx))):
	# feature matching
	for y in range(box[1]-int(padding*dy),box[1]+box[3]+int(padding*dy),int(stride*(box[3]+2*padding*dy))):
		#if (np.sum(mask[0][y:y+dy,x:x+dx]) < 3) and (padding != 0): # skip if see too much bullshit
		#	continue 
		template = main_img[y:y+dy,x:x+dx,:]
		min_val = [1e100,None]
		score_data =[]
		for i in range(640):
			if dx+i < 640:
				#if (np.sum(mask[1][y:y+dy,i+x:i+x+dx]) < 3) and (padding != 0): # skip if see too much bullshit
				#	continue
				search = search_img[y:y+dy,i:i+dx,:]
				score = np.average(np.power(template-search,2)) # mean square difference
				score_data.append(score)
				min_val = [score,i] if (score < min_val[0]) else min_val
		if min_val[1] != None:
			if (diff1 < diff2):
				matches.append([x + dx/2,y + dy/2,min_val[1] + dx/2,y + dy/2,-min_val[0]])
			else:
				matches.append([min_val[1] + dx/2,y + dy/2,x + dx/2,y + dy/2,-min_val[0]])

matches = Sort(matches,4)
ri = 0

contours, _ = cv2.findContours(rotation_mask.copy().astype(np.uint8), 1, 1) # not copying here will throw an error
rect = cv2.minAreaRect(contours[0]) # basically you can feed this rect into your classifier
(x,y),(w,h), a = rect # a - angle
boxy = cv2.boxPoints(rect)
boxy = np.int0(boxy) #turn into ints
rect2 = cv2.drawContours(rotation_img.astype(np.uint8).copy(),[boxy],0,(0,0,255),3)
cv2.imwrite("step/step8_roratedbox.png",rect2)
a = -(90-a) if w <= h else a
m_min = np.array([0.0,0.0,0.0,0.0])+10000
m_max = np.array([0.0,0.0,0.0,0.0])-10000
m_avg = np.array([0.0,0.0,0.0,0.0])
m_n = 0
for x1,y1,x2,y2,s in matches:
	area1 = cv2.rectangle(org[0], (int(x1-20),int(y1-20)), (int(x1+20),int(y1+20)), (0,0,255),3)
	area2 = cv2.rectangle(org[1], (int(x2-20),int(y2-20)), (int(x2+20),int(y2+20)), (0,0,255),3)
	org[0][int(y1),int(x1),:] = np.array([0,0,255])
	org[1][int(y2),int(x2),:] = np.array([0,0,255])
	# reconstruction
	Q = np.array([
		[1,0,0,-640/2],
		[0,1,0,-480/2],
		[0,0,0,f],
		[0,0,-1/tx,0]])
	X = np.array([[x1],[y1],[x2-x1],[1]])
	M = np.matmul(Q,X)
	m = np.transpose(np.array([M[0],M[1],M[2],a*M[3]])/M[3])[0]
	#m = (1-dx1)*m+(1-dx2)*m2
	if ( np.sum(np.isinf(m)+np.isnan(m)) < 1) :
		for i in range(4):
			if m[i] > m_max[i]:
				m_max[i] = m[i]
			if m[i] < m_min[i]:
				m_min[i] = m[i]
		#m_avg += m
		#m_n += 1
	ri += 1
	if ri > nfeature*len(matches):
		break

m_avg = 0.5*(m_min+m_max)
writer.writerow(m_avg.tolist())	
writer.writerow(["x", "x", "x", "x"])




