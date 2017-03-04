###imports
import cv2
import sys
import socket
import math
import time
import serial
import numpy as np
import Tkinter as tk
from Tkinter import *
from PIL import Image
from PIL import ImageTk
from tkinter import messagebox
import tkinter.ttk as ttk
import serial.tools.list_ports
#--------------------------------------------------------
 
 
###GUI
window = tk.Tk()  #Makes main window
window.wm_title("Head Tracking V1")
window.config(background="#FFFFFF")
 
imageFrame = tk.Frame(window, width=640, height=480)
imageFrame.grid(row=0, column=0, padx=5, pady=5)
 
imageFrame = Label(imageFrame)
imageFrame.grid(row=20, column=20)
#--------------------------------------------------------
 
 
###CAM properties
cam_width  = 640
cam_height = 480
cam_fps = 30
 
#Default value for cropping (eg. no cropping)
cropping_width_def_val  = cam_width
cropping_height_def_val = cam_height
cropping_pos_x_def_val = 0
cropping_pos_y_def_val = 0
 
faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
 
camera = cv2.VideoCapture(0)
#Give some time for the camera to start
time.sleep(1)
 
if (camera.isOpened()):
    print ("Camera opened")
 
#Set camera resolution and frame/sec if possible
camera.set(cv2.CAP_PROP_FRAME_WIDTH, cam_width)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_height)
camera.set(cv2.CAP_PROP_FPS, cam_fps)
#--------------------------------------------------------
 
 
### Head Tracking parameters
HeadMinSize_def_val  = 80
HeadMaxSize_def_val  = 600
minNeighbors_def_val = 5
scaleFactor_def_val  = 1.10
 
Socket_val  =  BooleanVar()
Serial_val =  BooleanVar()
 
HeadMinSize_val =  HeadMinSize_def_val
HeadMaxSize_val =  HeadMaxSize_def_val
minNeighbors_val = minNeighbors_def_val
scaleFactor_val  = scaleFactor_def_val
 
samples = 0
sum_x = 0
sum_y = 0
X_EyeBox = 999
Y_EyeBox = 999
 
X_EyeBox_val = IntVar(value=X_EyeBox)
Y_EyeBox_val = IntVar(value=Y_EyeBox)
 
EB_color = (255, 100, 50)
Head_color = (255, 255, 255)
Overlay1_color = (180, 180, 180)
Overlay2_color = (220, 220, 220)
#--------------------------------------------------------
 
 
###Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
# Connect the socket to the port on the server given by the caller
server_address = ('localhost', 10001)
print >>sys.stderr, 'connecting to %s port %s' % server_address
 
try:
    sock.connect(server_address)
    print >>sys.stderr, 'connected to %s port %s' % server_address
#except socket.timeout:
except:
    #print >>sys.stderr, 'connection timeout on %s port %s' % server_address
    print "Socket connection exception"
    sock.close()
#--------------------------------------------------------
 
 
### Serial Communication
def find_between( s, first, last ):
    try:
        start = s.index( first ) + len( first )
        end = s.index( last, start )
        return s[start:end]
    except ValueError:
        return "#err#"
 
 
portslist=[]
def serial_ports():    
    ports = serial.tools.list_ports.comports()
    for p in ports:
        portNumber = find_between(str(p), " (",")'")
        portslist.append(portNumber)
    return portslist
     
ser = serial.Serial()
ser.baudrate = 115200
ser.timeout = 0
 
def on_select(event=None):
 
    ser.port = Combo.get()
     
    try: 
        ser.open()
        print(ser.name + ' is open at ' + str(ser.baudrate))
    except:
        print "error open serial port"
        ser.close()
 
 
 
def Head_Tracking(Px, Py):
 
    global samples
    global sum_x
    global sum_y 
 
    #Get the serial and socket checkbutton value to send the coordinates
    Serial_val = Serialbutton.var.get()
    Socket_val = Socketbutton.var.get()
 
    #Check if the face coordinate are in range
    if( (Px!=999) & (Py!=999) ):
     
        X_EyeBox = Px
        Y_EyeBox = Py
  
    else:
        X_EyeBox = 999
        Y_EyeBox = 999
 
 
    if(samples == 10):
        result_x = int(sum_x/samples)
        result_y = int(sum_y/samples)
 
        #Display eye box coordinate on GUI
        X_EyeBox_val.set(int(result_x))
        Y_EyeBox_val.set(int(result_y))
 
        #Output buffer
        serout = ('<%+0.3d|%+0.3d>' % (result_x, result_y) )
 
        if(Socket_val == 1):
            try:
                sock.send(serout)
            except:
                sock.close()
 
        if( Serial_val == 1 ):
            try:                
                ser.write(serout)
                print(serout)
            except Exception, e:
                print "Write exception on serial port: " + str(e)
 
            time.sleep(0)
 
        sum_x = 0
        sum_y = 0
        samples = 0
 
    else:
        global sum_x
        global sum_y
 
        #Only average correct values
        if( (X_EyeBox != 999) & (Y_EyeBox != 999) ):
            sum_x = sum_x + X_EyeBox
            sum_y = sum_y + Y_EyeBox
            samples = samples+1
             
        sum_x = sum_x
        sum_y = sum_y
         
    time.sleep(0.01)
 
 
 
def VideoCapture():
    #Get the parameters value for face tracking from GUI
    HeadMinSize_val =  HeadMinSizeScaler.get()
    HeadMaxSize_val =  HeadMaxSizeScaler.get()
    minNeighbors_val =  minNeighborsScaler.get()
    scaleFactor_val =  scaleFactorScaler.get()
 
    #Get the parameters value for cropping from GUI
    CroppingWidth_val = CroppingWidthScaler.get()
    CroppingHeight_val = CroppingHeightScaler.get()
    CroppingXPos_val = cropping_pos_x_def_val
    CroppingYPos_val = cropping_pos_y_def_val
     
    if(camera.isOpened() ):
 
        #Read video stream from camera
        ret, frame = camera.read()
 
        #Set the proper cropping value
        x1 = ((cam_width - CroppingWidth_val)/2)
        y1 = ((cam_height - CroppingHeight_val)/2)
        x2 = (x1 + CroppingWidth_val)
        y2 = (y1 + CroppingHeight_val)
 
        #Crop the video frame to the new format
        crop = frame[y1:y2, x1:x2]
         
        #Use cropped frame in gray mode
        frameGray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) 
 
        #Display the new resolution to work with
        cv2.putText(crop,"{}x{}".format(CroppingWidth_val, CroppingHeight_val),(5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220,220,220), 1, cv2.LINE_AA)
 
        #Face detection
        face = faceCascade.detectMultiScale(
            frameGray,
            scaleFactor= scaleFactor_val,
            minNeighbors= minNeighbors_val,
            minSize=(HeadMinSize_val, HeadMinSize_val),
            maxSize=(HeadMaxSize_val, HeadMaxSize_val),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
         
        #Draw a rectangle around the face - detecting only one face   
        if len(face) == 1:
            for (hx, hy, hw, hh) in face:
                cv2.rectangle(crop, (hx, hy), (hx+hw, hy+hh), Head_color, 2)        #Draw rectanlge around the face
                headpos_x = (CroppingWidth_val/2) - (hx+hh/2)                       #Get the Head x coordinate
                headpos_y = (CroppingHeight_val/2) - (hy+hw/2)                      #Get the Head y coordinate
                cv2.circle(crop, (hx+(hw/2), int(hy+(hh*0.37))), 10, EB_color, 2)   #Draw a circle on the eye box position
                EB_Pos_x = (headpos_x)                                              #Extract eye box x position
                EB_Pos_y = (headpos_y + int( (hh/2)-(hh*0.37)))                     #Extract eye box y position (0.37 = magic number)
        else:
            #no face detected
            EB_Pos_x = 999
            EB_Pos_y = 999
                 
        #Display center lines + eye bos position
        cv2.line(crop, (0, CroppingHeight_val/2), (CroppingWidth_val, CroppingHeight_val/2), Overlay1_color, 1, cv2.LINE_AA)
        cv2.line(crop, (CroppingWidth_val/2, 0), (CroppingWidth_val/2, CroppingHeight_val), Overlay1_color, 1, cv2.LINE_AA)
        cv2.rectangle(crop, (CroppingWidth_val/2-10, CroppingHeight_val/2-10), (CroppingWidth_val/2+10, CroppingHeight_val/2+10), Overlay1_color, 1)
        cv2.putText(crop,"EB Pos : x{} y{}".format(int(EB_Pos_x), int(EB_Pos_y)),(5,(CroppingHeight_val-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, Overlay2_color, 1, cv2.LINE_AA)
         
        #Set back the frame to color to display on GUI
        color = cv2.cvtColor(crop, cv2.COLOR_BGR2RGBA)
 
        #TkInter process for displaying the video
        img = Image.fromarray(color)
        imgtk = ImageTk.PhotoImage(image=img)
        display1.imgtk = imgtk
        display1.configure(image=imgtk)
 
        #Process the Eye box position
        Head_Tracking(Px=EB_Pos_x, Py=EB_Pos_y)
 
    window.after(5, VideoCapture)
 
     
     
def HeadResetParameters():
    HeadMinSizeScaler.set(HeadMinSize_def_val)
    HeadMaxSizeScaler.set(HeadMaxSize_def_val)
    minNeighborsScaler.set(minNeighbors_def_val)
    scaleFactorScaler.set(scaleFactor_def_val)
    CroppingWidthScaler.set(cam_width)
    CroppingHeightScaler.set(cam_height)
 
     
     
     
###Control Frame
Controls = LabelFrame(imageFrame, text='Controls', font = "Consolas 11 bold")
Controls.grid(row=0, column=0, rowspan=10, ipadx=5, ipady=5, sticky='nesw')
 
###BlankSpace
blankspace = Label(Controls)
blankspace.grid(row=0, column=0, sticky='we')
 
###HitBoxFrame
HitBoxFrame = LabelFrame(Controls, text='HitBox', font = "Consolas 10 bold")
HitBoxFrame.grid(row=1, column=0 ,sticky='nesw', ipadx=5, ipady=5, columnspan=1)
Label(HitBoxFrame, text="Min Size", font = "Consolas 10").grid(row=0, column=1, sticky='ws')
HeadMinSizeScaler = Scale(HitBoxFrame, from_=20, to=200, resolution=10, orient=HORIZONTAL)
HeadMinSizeScaler.grid(row=0, column=0, sticky='w')
HeadMinSizeScaler.set(HeadMinSize_def_val)
Label(HitBoxFrame, text="Max Size", font = "Consolas 10").grid(row=1, column=1, sticky='ws')
HeadMaxSizeScaler = Scale(HitBoxFrame, from_=200, to=600, resolution=10, orient=HORIZONTAL)
HeadMaxSizeScaler.grid(row=1, column=0, sticky='w')
HeadMaxSizeScaler.set(HeadMaxSize_def_val)
 
###Cropping Frame
CroppingFrame = LabelFrame(Controls, text='Cropping', font = "Consolas 10 bold")
CroppingFrame.grid(row=2, column=0 ,sticky='nesw', ipadx=5, ipady=5, columnspan=1)
Label(CroppingFrame, text="Width ", font = "Consolas 10").grid(row=0, column=1, sticky='ws')
CroppingWidthScaler = Scale(CroppingFrame, from_=int(cam_width/2), to=cam_width,  resolution=10,orient=HORIZONTAL)
CroppingWidthScaler.grid(row=0, column=0, sticky='w')
CroppingWidthScaler.set(cropping_width_def_val)
Label(CroppingFrame, text="Height", font = "Consolas 10").grid(row=1, column=1, sticky='ws')
CroppingHeightScaler = Scale(CroppingFrame, from_=int(cam_height/2), to=cam_height,  resolution=10,orient=HORIZONTAL)
CroppingHeightScaler.grid(row=1, column=0, sticky='w')
CroppingHeightScaler.set(cropping_height_def_val)
 
###SpecialFrame
SpecialFrame = LabelFrame(Controls, text='Special', font = "Consolas 10 bold")
SpecialFrame.grid(row=3, column=0 ,sticky='nesw', ipadx=5, ipady=5)
Label(SpecialFrame, text="MinNeighbors", font = "Consolas 10").grid(row=0, column=1, sticky='ws')
minNeighborsScaler = Scale(SpecialFrame, from_=1, to=10, orient=HORIZONTAL)
minNeighborsScaler.grid(row=0, column=0, sticky='we')
minNeighborsScaler.set(minNeighbors_def_val)
Label(SpecialFrame, text="ScaleFactor", font = "Consolas 10").grid(row=1, column=1, sticky='ws')
scaleFactorScaler = Scale(SpecialFrame, from_=1.02, to=1.6, resolution=0.02, orient=HORIZONTAL)
scaleFactorScaler.grid(row=1, column=0, sticky='we')
scaleFactorScaler.set(scaleFactor_def_val)
 
### HeadResetbutton
HeadResetbutton = Button(Controls, width=16, height=1, text='Reset params', font = "Consolas 10", fg="red", command = HeadResetParameters)
HeadResetbutton.grid(row=4, column=0, padx=2, pady=2, sticky='nesw')
 
###BlankSpace
blankspace = Label(Controls)
blankspace.grid(row=5, column=0, sticky='we')
 
###SerialComFrame
SerialComFrame = LabelFrame(Controls, text='SerialCom', font = "Consolas 10 bold")
SerialComFrame.grid(row=6, column=0, sticky='nesw', ipadx=5, ipady=5, columnspan=1)
Serialbutton = Checkbutton(SerialComFrame, width=20, height=1, text='Send position', font = "Consolas 10", variable = Serial_val)
Serialbutton.grid(row=0, column=1, sticky='ws')
Serialbutton.var = Serial_val
Serialbutton.var.set(0)
Combo = ttk.Combobox(SerialComFrame, width=10, values=serial_ports())
Combo.grid(row=0, column=0, sticky='ws')
Combo.set("none")
 
###SocketFrame
SocketFrame = LabelFrame(Controls, text='SocketCom', font = "Consolas 10 bold")
SocketFrame.grid(row=7, column=0, sticky='nesw', ipadx=5, ipady=5, columnspan=1)
Socketbutton = Checkbutton(SocketFrame, width=20, height=1, text='Send position', font = "Consolas 10", variable = Socket_val)
Socketbutton.grid(row=0, column=1, sticky='ws')
Socketbutton.var = Socket_val
Socketbutton.var.set(0)
 
### CameraFrame
CameraFrame = LabelFrame(imageFrame, width=cam_width, height=cam_height)
CameraFrame.grid(row=2, column=1, padx=5, pady=5, columnspan=2)
imagePreviewFrame = tk.Frame(CameraFrame, width=cam_width, height=cam_height)
imagePreviewFrame.grid(padx=5, pady=5, sticky='nesw')
imagePreviewFrame.grid_propagate(0)
display1 = tk.Label(imagePreviewFrame)
display1.grid(row=0, column=0, sticky='nesw')
display1.place(in_=imagePreviewFrame, anchor="c", relx=.5, rely=.5)
 
###EyeBox Frame
EyeBoxFrame = LabelFrame(imageFrame, text='EyeBox Position', font = "Consolas 11 bold")
EyeBoxFrame.grid(row=0, column=1, columnspan=2, sticky='nesw')
Label(EyeBoxFrame, text=" X =", font = "Consolas 12 bold").grid(row=0, column=0, sticky=W)
Label(EyeBoxFrame, text=" Y =", font = "Consolas 12 bold").grid(row=1, column=0, sticky=W)
Label(EyeBoxFrame, textvariable=X_EyeBox_val, font = "Consolas 12 bold").grid(row=0, column=1, sticky=W)
Label(EyeBoxFrame, textvariable=Y_EyeBox_val, font = "Consolas 12 bold").grid(row=1, column=1, sticky=W)
 
 
#When closing application
def on_closing():
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        sock.close()
        camera.release()
        time.sleep(1)
        cv2.destroyAllWindows()
        window.destroy()
 
Combo.bind('<<ComboboxSelected>>', on_select)
window.protocol("WM_DELETE_WINDOW", on_closing)
VideoCapture()      #Face recognition
window.mainloop()   #Starts GUI
#End of code