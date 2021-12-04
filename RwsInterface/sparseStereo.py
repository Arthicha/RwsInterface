# system module
import os, sys, inspect
import csv
os.chdir(os.path.dirname(os.path.abspath(__file__)))
print(os.path.dirname(os.path.abspath(__file__)))
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
from numpy import linalg as LA
import matplotlib.pyplot as plt 
# feature extraction models
import feature_ext_models.CARN.carn as carn


MODEL = "CARN"

if len(sys.argv) != 8:
	print("ERROR: input arguments do not match!")
	sys.exit()

method = int(sys.argv[1])
f = float(sys.argv[2])
tx = float(sys.argv[3])
window = float(sys.argv[4])
stride = float(sys.argv[5])
padding = float(sys.argv[6])
nfeature = float(sys.argv[7])

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
	mask[120:380,:,:] = np.array([255,255,255])
	diff = LA.norm(img-null,axis=2)
	mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
	mask = np.where(diff < 10,0.0,255.0)
	mask[:120,:] = 0
	return mask

def getBoxes(mask):
	# calculate the bounding box
	thresh = np.uint8(mask)
	cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if len(cnts) == 2 else cnts[1]
	boxes = []
	for c in cnts:
		x,y,w,h = cv2.boundingRect(c)
		boxes.append([x,y,w,h])
	return boxes

def Sort(sub_li,elem_id):
    # reverse = None (Sorts in Ascending order)
    # key is set to sort using second element of 
    # sublist lambda has been used
    return(sorted(sub_li, key = lambda x: x[elem_id])) 

print("INFO: python script starts!")



# data preparation
org = [cv2.flip(cv2.imread("img"+str(i)+".png"), 1) for i in range(2)]
null = [cv2.flip(cv2.imread("null"+str(i)+".png"), 1) for i in range(2)]
mask = [getMask(org[i],null[i]) for i in range(2)]
img = [org[i] * np.stack([mask[i],mask[i],mask[i]],axis=2)/255 for i in range(2)]
boxes = [getBoxes(mask[i]) for i in range(2)]



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
box = boxes[0][0]
dx = int(box[2]*window)-1 if window != 0.0 else 1
dy = int(box[3]*window)-1 if window != 0.0 else 1

matches = []
for x in range(box[0]-int(padding*dx),box[0]+box[2]+int(padding*dx),int(stride*(box[2]+2*padding*dx))):
	print(x,"/",box[0]+box[2]+dx)
	# feature matching
	for y in range(box[1]-int(padding*dy),box[1]+box[3]+int(padding*dy),int(stride*(box[3]+2*padding*dy))):
		if (np.sum(mask[0][y:y+dy,x:x+dx]) < 3) and (padding != 0): # skip if see too much bullshit
			continue 
		template = feature[0][y:y+dy,x:x+dx,:]
		min_val = [1e100,None]
		score_data =[]
		for i in range(300):
			if x+dx+i < 640:
				if (np.sum(mask[1][y:y+dy,i+x:i+x+dx]) < 3) and (padding != 0): # skip if see too much bullshit
					continue
				search = feature[1][y:y+dy,i+x:i+x+dx,:]
				score = np.average(np.power(template-search,2)) # mean square difference
				score_data.append(score)
				min_val = [score,i] if (score < min_val[0]) else min_val
		if min_val[1] != None:
			matches.append([x + dx/2,y + dy/2,min_val[1]+x + dx/2,y + dy/2,min_val[0]])

matches = Sort(matches,4)
ri = 0

contours, _ = cv2.findContours(mask[0].copy().astype(np.uint8), 1, 1) # not copying here will throw an error
rect = cv2.minAreaRect(contours[0]) # basically you can feed this rect into your classifier
(x,y),(w,h), a = rect # a - angle
boxy = cv2.boxPoints(rect)
boxy = np.int0(boxy) #turn into ints
rect2 = cv2.drawContours(img[0].astype(np.uint8).copy(),[boxy],0,(0,0,255),3)
cv2.imwrite("box1.png",rect2)
a = -(90-a) if w <= h else a
m_avg = np.array([0.0,0.0,0.0,0.0])
m_n = 0
for x1,y1,x2,y2,s in matches:
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
	if ( np.sum(np.isinf(m)+np.isnan(m)) < 1) :
		m_avg += m
		m_n += 1
	ri += 1
	if ri > nfeature*len(matches):
		break
m_avg = m_avg/m_n
writer.writerow(m_avg.tolist())	
writer.writerow(["x", "x", "x", "x"])

for i in range(2):
	cv2.imwrite("output"+str(i+1)+".png",org[i])



