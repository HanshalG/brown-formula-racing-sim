import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from scipy import ndimage
from scipy.signal import convolve2d, convolve, find_peaks
from math import acos
from scipy.interpolate import UnivariateSpline as scipy_spline
import cv2 as cv

track_image = io.imread('image6.png')
temp = np.zeros(track_image.shape)
temp[:,:,0] += -255
dist = np.linalg.norm(track_image+temp,axis = 2)
dist = dist/np.max(dist)
thresh = .4
dist[dist>thresh] = 1
track_width_px = 15
filter = np.ones([track_width_px,track_width_px])
for i in range(track_width_px):
    for j in range(track_width_px):
        x = i-int(track_width_px/2)
        y = j-int(track_width_px/2)
        r = (x**2+y**2)**(1/2)
        if r>int(track_width_px/2):
            filter[i,j] = 0
filter = filter/np.sum(filter.flatten())
conv = convolve2d(dist,filter,mode = 'valid')
thresh_2 = .4
conv[conv>thresh_2] = 1

resolution = 50
start = [2920+resolution-1,80+resolution-1]
continue_bool = True
coords = [start]
curr_coords = start
x_min = 0
x_max = conv.shape[1]
y_min = 0
y_max = conv.shape[0]


conv = np.pad(conv,pad_width = resolution,mode = 'constant',constant_values = 1)
while continue_bool:
    x_lower = curr_coords[1]-resolution
    x_upper = curr_coords[1]+resolution
    y_lower = curr_coords[0]-resolution
    y_upper = curr_coords[0]+resolution
    conv[curr_coords[0],curr_coords[1]]= 1
    
    temp = conv[y_lower:y_upper+1,x_lower:x_upper+1]
    temp[1:resolution*2-1,1:resolution*2-1] = 1
    values = temp !=1
    if np.sum(values.flatten()) == 0:
        continue_bool = False
    max_ind = np.unravel_index(np.argmin(temp, axis=None), temp.shape)+np.array([-resolution,-resolution])
    curr_coords = curr_coords+max_ind
    conv[y_lower:y_upper+1,x_lower:x_upper+1] = np.ones([resolution*2+1,resolution*2+1])

    coords.append(curr_coords)
coords = np.array(coords)

fig, ax = plt.subplots()
conv_factor = 1400/max(track_image.shape)
track_points = np.array([coords[:,1]+resolution,-coords[:,0]+max(coords[:,0])+resolution])
track_points = track_points*conv_factor
ax.imshow(track_image, extent=[0, int(track_image.shape[1]*conv_factor), 0, int(track_image.shape[0]*conv_factor)])

xs = track_points[0,:]
ys = track_points[1,:]
t = np.arange(xs.shape[0])
t_res = np.arange(xs.shape[0]*100)/100

fit_x = scipy_spline(t,xs,k=3,s = 50)
fit_y = scipy_spline(t,ys,k=3, s = 50)
plt.scatter(xs,ys)

# plt.plot(t_res,fit_x(t_res))
# plt.plot(t_res,fit_y(t_res))
# plt.scatter(t,xs)
# plt.scatter(t,ys)
# plt.show()

xp = fit_x.derivative(1)(t_res)
xpp = fit_x.derivative(2)(t_res)
yp = fit_y.derivative(1)(t_res)
ypp = fit_y.derivative(2)(t_res)
k = (xp*ypp-yp*xpp)/(xp**2+yp**2)**(3/2)
k[k==0] = .0001

r = 1/k
radii_threshold = 200 #max speed = 65 mph
r[r>radii_threshold] = radii_threshold
r[r<-radii_threshold] = -radii_threshold
apexes = find_peaks(max(r)-np.abs(r))[0]
crit_points = find_peaks(np.abs(r))[0]
plt.scatter(fit_x(t_res[apexes]),fit_y(t_res[apexes]),c = r[apexes])
plt.plot(fit_x(t_res),fit_y(t_res))
#plt.colorbar()
plt.show()
track_list = []
curr_straight = True
before_apex = True
curr_len = 0
abs_r = abs(r)
curr_r = radii_threshold
curr_r_i = 1
for i in range(1,len(t_res)):
    if abs_r[i] == radii_threshold:
        if not curr_straight:
            before_apex = True
            string = 'Left'
            if r[i]<0:
                string = 'Right'
            track_list.append([string,curr_len,abs(curr_r),curr_r_i])
            curr_r = 0
            curr_len = 0
            curr_r_i = i
        curr_straight = True
    else:
        if curr_straight:
            track_list.append(['Straight',curr_len,0,curr_r_i])
            curr_len = 0
            curr_straight = False
            before_apex = True
        elif before_apex:
            if abs_r[i-1]<abs_r[i]:
                curr_r_i = i
                before_apex = False
                curr_r = r[i-1]
        else:
            if abs_r[i-1]>abs_r[i]:
                before_apex = True
                string = 'Left'
                if r[i]<0:
                    string = 'Right'
                track_list.append([string,curr_len,abs(curr_r),curr_r_i])
                curr_len = 0
    p1 = np.array([fit_x(t_res[i-1]),fit_y(t_res[i-1])])
    p2 = np.array([fit_x(t_res[i]),fit_y(t_res[i])])
    dist_vect = p1-p2
    dist = np.linalg.norm(dist_vect)
    curr_len += dist
track_list.append(['Straight',curr_len,0,curr_r_i])

track_list = np.array(track_list)
plt.plot(t_res,abs(r))
plt.scatter(t_res[apexes],abs(r[apexes]))
plt.scatter(t_res[track_list[:,3].astype(int)],track_list[:,2].astype(float))
plt.show()
np.savetxt('test.csv', track_list, delimiter=',', fmt='%s')
