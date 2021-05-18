#!/usr/bin/env python
# coding=utf-8

# To do:
# 
# raspi kühler (aktiv)
# nicht nur greifer runter und hoch, sondern auch noch Mittelding für schwarze Ecke
# autostart von Linefollowerprogramm
# prüfen, ob auch wirklich eine Kugel aufgenommen wurde
# schnellere baudrate
# raspi übertakten
# Bei Lücke ein Stückchen in die richtige Richtung drehen (ein paar Werte, bevor weiß kam schauen, ob Linienpos rechts oder links war und dann ein Stück koriggieren)
# Grüne Punkte besser erkennen
# Dose umfahren und sich dabei nicht von anderen Linien irritieren lassen (neues ROI, ganz links am Kamerabild bzw. einfach alles rechts abschneiden)
# T Platte schaffen
# Silber erkennen verbessern
# Rescue Kit aufnehmen können
# Rescue Kit erkennen können (richtige Farbwerte herausfinden!!!)
# Rescue Kit am Anfag des Rescuebereichs ablegen
# Lebendes und totes Opfer unterscheiden
# Kugeln einzeln suchen und zur Ecke bringen
# Ausgang des Rescuebereichs finden 
# Ende des Parkours erkennen

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import argparse
import time
import cv2
import serial
import random
import os

CUT = (50, 270, 120, 192) #eigentlich (50, 270, 120, 192)
CUT_GRN = (50, 270, 80, 192) #eigentlich (50, 270, 120, 192)
CUT_SILVER = (0, 100, 0, 180)
CUT_RESCUEKIT = (50, 270, 120, 170)

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout = 2) #USB "Adresse" und Baudrate des Arduinos

while(not ser.is_open):
	print("Waiting for Serial...")
	time.sleep(0.1)
print("Opened:", ser.name, "aka Teensy 3.6") 

camera = PiCamera()
camera.resolution = (320, 192)
camera.rotation = 0
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 192))
rawCaptureCircles = PiRGBArray(camera, size=(320, 192))

time.sleep(0.2) #wartet kurz, damit Arduino und Kamera bereit sind
framesTotal = 0 #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
startTime = time.time() #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
timeWaitet = 0 # zählt, wie lange time.sleep im ganzen Programm insgesamt aufgerufen wurden und zieht diese Zeit bei Berechnung der FPS ab
lastLinePos = 0
LinePosLastLoop = [0, 0, 0, 0, 0, 0, 0, 0]
value = 0
gapcounter = 0
grn_list = []
grn_counter = 0
rescueCounter = 0
rescue = False
mindist = 300 


x = 0
y = 0
r = 0

##########FUNKTIONEN#############

def DEBUG():
	cv2.imshow("image_rgb", image_rgb) #gibt das aktuelle Bild aus
	cv2.imshow("image_hsv", image)
	cv2.imshow("cut", cut)
	cv2.imshow("cut_green", cut_grn)
	#cv2.imshow("cut_silber", cut_silver)
	cv2.imshow("rescuekit", rescuekit)
	#cv2.imshow("Konturen gruen", green)
	cv2.setMouseCallback("mouseRGB", mouseRGB)
	cv2.imshow("mouseRGB", image_rgb)
	
def DEBUG_LastLinePos():
	print("LinePosLastLoop[0] = ", LinePosLastLoop[0])
	print("LinePosLastLoop[1] = ", LinePosLastLoop[1])
	print("LinePosLastLoop[2] = ", LinePosLastLoop[2])
	print("LinePosLastLoop[3] = ", LinePosLastLoop[3])
	print("LinePosLastLoop[4] = ", LinePosLastLoop[4])
	print("LinePosLastLoop[5] = ", LinePosLastLoop[5])
	print("LinePosLastLoop[6] = ", LinePosLastLoop[6])
	print("LinePosLastLoop[7] = ", LinePosLastLoop[7])

def mouseRGB(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = image_rgb[y, x, 0]
        colorsG = image_rgb[y, x, 1]
        colorsR = image_rgb[y, x, 2]
        colors = image_rgb[y, x]
        """
        print("Red: ", colorsR)
        print("Green: ", colorsG)
        print("Blue: ", colorsB)
        print("BRG Format: ", colors)
        print("Coordinates of pixel: X: ", x,"Y: ", y)
        """
        colour = np.uint8([[[colorsB, colorsG, colorsR]]])
        colour_hsv = cv2.cvtColor(colour, cv2.COLOR_BGR2HSV)
        print(colour_hsv)



def delay(zeit):
	global timeWaitet
	zeit = float(zeit)
	time.sleep(zeit)
	timeWaitet = timeWaitet + zeit
	print(timeWaitet)

def fahre(motorLinks, motorRechts, zeit):
	send = str(motorLinks) + ':' + str(motorRechts) + ':' + str(zeit)
	print("Send:", send)
	ser.write(send.encode())
	zeit = float(zeit / 1000.0)
	time.sleep(0.1)
	while True:
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1":
			break

def drehe(deg):
	fahre(0, 0, deg)

def greiferRunter():
	sendeUndWarteAufEmpfang("greiferRunter")

def greiferHoch():
	sendeUndWarteAufEmpfang("greiferHoch")

def sendeUndWarteAufEmpfang(send):
	ser.write(send.encode())
	while True:
		readData = ser.readline().decode('ascii').rstrip()
		if readData == "1":
			break

def sucheSchwarzeEcke(pIsWallRight):
	if pIsWallRight == True:
		print("suche Ecke mit Wand rechts")
	else:
		print("suche Ecke mit Wand links")
		sendeUndWarteAufEmpfang("fahreZuEckeUndLadeKugelAb")

def sucheAusgang(pIsWallRight):	
	camera = PiCamera()
	camera.resolution = (320, 192)
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 192))
	if pIsWallRight == True:
		print("suche Ausgang mit Wand rechts")
	else:
		print("suche Ausgang mit Wand links")
		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
			image = frame.array #speichert das aktuelle Bild der Kamera in Variable ab
			image = image[50:270][50:192]			
			image_rgb = image

			image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentu
			image = cv2.GaussianBlur(image, ((15, 15)), 2, 2)


			green = cv2.inRange(image, (30, 20, 20), (100, 255, 255)) # Kalibrierung gruen	eigentlich (55, 40, 40), (80, 255, 255)

			contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
			print(len(contours_grn))
			if(len(contours_grn) > 0):
				cv2.imshow("Exit", image_rgb)
				cv2.putText(image_rgb, "Exit", (110, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 106, 255), 3)
				fahre(130, 130, 1000)
				camera.close()
				cv2.destroyAllWindows()
				os.system('python3 test.py')
				return
			else:
				fahre(255, 255, 50)
			cv2.drawContours(image_rgb, contours_grn, -1, (0, 106, 255), 3)
			cv2.imshow("Exit", image_rgb)
			rawCapture.truncate(0)
			key = cv2.waitKey(1) & 0xFF
			if key == ord("q"):
				break
def rescue():
	keineKugelDa = 0 #zählt, in vielen Frames (in Folge) keine Kugel vorhanden war
	turnCnt = 0 #zählt, um wie viel grad sich der raspi schon gedreht hat
	umdrehungenCnt = 0 #zählt, wie oft schon um 360° gedreht wurde

	camera = PiCamera()
	camera.resolution = (320, 180) #Aufloesung, je niedriger desto schneller das Program
	camera.rotation = 0
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(320, 180))
	print("Rescue program started")
	time.sleep(1)
	framesTotalRescue = 0 #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen
	startTimeRescue = time.time() #erstellt er, um bei Eingabe von q die durchsch. FPS anzeigen zu koennen

	isWallRigth = False #wo ist eine Wand im Rescuebereich?
	fahre(255, 200, 1000)
	drehe(80)
	fahre(-255, -255, 500)
	sendeUndWarteAufEmpfang("setzeUrsprung") #Roboter ist gerade an einer Wand ausgerichtet und merkt sich daher die pos, um später wieder dorthin drehen zu können
	fahre(255, 255, 300)
	drehe(-90)
	fahre(255, 255, 900)
	drehe(45)
	fahre(255, 255, 650)
	drehe(90)
	fahre(-255, -255, 800)	
	greiferRunter()
	fahre(255, 255, 20)
	fahre(-255, -255, 100)
	fahre(255, 255, 20)
	fahre(-255, -255, 100)
	greiferHoch()
	fahre(255, 255, 1500)
	sendeUndWarteAufEmpfang("dreheZuUrsprung")
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		image = cv2.GaussianBlur(image, ((5, 5)), 2, 2)
		
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp = 1, minDist = 60, param1 = 34, param2 = 24, minRadius = 2, maxRadius = 300)

		# ensure at least some circles were found
		if circles is not None:
			if keineKugelDa > 5: #da Kugel gefunden wurde, macht er den Cnt um 5 kleiner
				keineKugelDa = keineKugelDa - 5
			else:
				keineKugelDa = 0
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
				#print(y) # y = naehe bzw y Achse
				# draw the circle in the output image, then draw a rectangle
				cv2.circle(image, (x, y), r, (255, 255, 0), 4)
				cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 0, 255), -1)
				ballPosition = x - 160
				if ballPosition > -8 and ballPosition < 8: #Ball liegt Mittig (Horizontal)
					print(y)
					if y > 120 and y < 140: #perfekt ausgerichtet
						drehe(180)
						fahre(-255, -255, 30)
						greiferRunter()
						greiferHoch()
						#suche schwarze Ecke:
						sendeUndWarteAufEmpfang("dreheZuUrsprung")
						time.sleep(3)
						sucheSchwarzeEcke(isWallRigth)
						drehe(20)
						fahre(255, 255, 550)
						drehe(-10)
						camera.close()
						sucheAusgang(isWallRigth)
						return
					elif y > 170:
						fahre(-255, -255, 30)
					elif y > 140:
						fahre(-255, -255, 10)
					elif y < 90:
						fahre(255, 255, 30)
					elif y < 120:
						fahre(255, 255, 10)
				elif ballPosition <= -8 and ballPosition >= -25:
					fahre(-150, 150, 20)
				elif ballPosition <= -25:
					fahre(-255, 255, 50)

				elif ballPosition >= 8 and ballPosition <= 25:
					fahre(150, -150, 20)
				elif ballPosition >= 25:
					fahre(255, -255, 50)
			cv2.putText(image, str(ballPosition), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 3)
		else:
			keineKugelDa = keineKugelDa + 1 #ein Frame ohne Kugel -> erhöhe Zähler
			if keineKugelDa >= 10: #10 mal in Folge keine Kugel gefunden -> Fahre ein Stücl weiter
				if turnCnt < 360:
					if umdrehungenCnt == 0:
						drehe(45)
						turnCnt = turnCnt + 45
					elif umdrehungenCnt == 1:
						fahre(255, 255, 300)
						drehe(60)
						turnCnt = turnCnt + 60
					else:
						print("Fertig mit allem, fahre bitte den Rand ab xD")
				else:
					print("Habe mich einmal um 360 grad gedreht!")
					#sendeUndWarteAufEmpfang("dreheZuUrsprung")
					umdrehungenCnt = umdrehungenCnt + 1
					turnCnt = 0
		cv2.imshow("Kugel output", image)
		rawCapture.truncate(0)
		key = cv2.waitKey(1) & 0xFF
		framesTotalRescue = framesTotalRescue + 1
		if key == ord("q"):
			print("Avg. FPS:", int(framesTotalRescue / (time.time() - startTimeRescue))) #sendet durchsch. Bilder pro Sekunde (FPS)
			camera.close()
			break #beendet Program bzw. bricht aus Endlosschleife aus
	print("Rescue Program beendet")

##############################################################################################
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array #speichert das aktuelle Bild der Kamera in Variable ab
	image_rgb = image 

	image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Konvertiert das Bild zum Christentu
	image = cv2.GaussianBlur(image, ((9, 9)), 2, 2)
	

	cut = image[CUT[0]:CUT[1]][CUT[2]:CUT[3]] # Teil des frames fuer die Linienerkennung ausschneiden
	cut_grn = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]] # Teil des frames fuer die Gruenerkennung ausschneiden (etwas groeßer)
	cut_silver = image[CUT_SILVER[0]:CUT_SILVER[1]][CUT_SILVER[2]:CUT_SILVER[3]]
	cut_rescuekit = image[CUT_GRN[0]:CUT_GRN[1]][CUT_GRN[2]:CUT_GRN[3]]
	
	cv2.GaussianBlur(cut_silver, ((9, 9)), 2, 2) #den Bereich für die Silbererkennung noch mal extra verschwimmen lassen	

	line = cv2.inRange(cut, (0, 0, 0), (255, 255, 75)) # Kalibrierung schwarz eigentlich (0, 0, 0), (255, 255, 75))
	green = cv2.inRange(cut_grn, (55, 200, 40), (80, 255, 255)) # Kalibrierung gruen	eigentlich (55, 40, 40), (80, 255, 255)
	silber = cv2.inRange(cut_silver, (0, 0, 0), (255, 255, 75))
	rescuekit = cv2.inRange(cut_rescuekit, (110, 230, 50), (130, 255, 150)) #richtige Werte eintragen




	contours_blk, hierarchy_blk = cv2.findContours(line.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_grn, hierarchy_grn = cv2.findContours(green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours_silver, hierarchy_silver = cv2.findContours(silber.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours_rescuekit, hierarchy_rescuekit = cv2.findContours(rescuekit.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	
	linePos = 0
	index = 0


	if len(contours_rescuekit) > 0:		
		ser.write(b'STOP')	
		print("SEND: STOP")
		exit()

	### Silbererkennung:
	if len(contours_silver) > 0: #falls die Kontur breiter als 0px ist / falls er Kontur findet
	   x_silber, y_silber, w_silber, h_silber = cv2.boundingRect(contours_silver[0]) # erstellt Rechteck um die Konturen 
	   #cv2.rectangle(image_rgb, (x_silber, y_silber), (x_silber + w_silber, y_silber + h_silber), (189, 189, 189), 3) #malt dieses Rechteck auch ins Bild
	   #print("kein silber erkannt")
	   if rescueCounter > 2:
	   	rescueCounter = rescueCounter - 3
	if len(contours_silver) == 0 :
		rescueCounter = rescueCounter + 1
		if rescueCounter > 10: #hat 10 mal in Folge kein schwarz gesehen -> da ist (wahrscheinlich)	der Rescue Bereich
			print("silber entgueltig erkannt")
			cv2.putText(image_rgb, "rescue", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
			#ser.write(b'Rescue') #sendet an den Teensy, dass er silber erkannt hat
			read_serial = ser.readline().decode('ascii') #schaut, ob Daten (in diesem Fall trigger fuer Rescuebereich) empfangen wurden
			if read_serial == '8\r\n': #da ist wirklich der Rescuebereich	
				cv2.destroyAllWindows()
				camera.close()
				rescue()
				print("Nach rescue Funktion")
				#Initialisiere Kamera:
				camera = PiCamera()
				camera.resolution = (320, 192)
				camera.rotation = 0
				camera.framerate = 32
				rawCapture = PiRGBArray(camera, size=(320, 192))
				rawCapture.truncate(0)
				print("nach Initialisierung")
			else:
				print("Der Teensy hat gesagt, dass es doch nicht der Rescue ist")
				ser.write(str(0/10).encode())
				rescueCounter = 0
	if(len(contours_blk) > 0):
		nearest = 1000
		for i in range(len(contours_blk)):
			b = cv2.boundingRect(contours_blk[i])
			x, y, w, h = b
			a = int(abs(x + w / 2 - 160 - lastLinePos))
			cv2.rectangle(image_rgb, (x, y + CUT[2] + CUT[0]), (x + w, y + h + CUT[2] + CUT[0]), (0, 106, 255), 2) #rechteck um schwarze Konturen
			if(a < nearest):
				nearest = a
				index = i
		b = cv2.boundingRect(contours_blk[index])
		x, y, w, h = b
		#print(w)
		if(w > 300): #falls sehr viel schwarz zu sehen ist, sendet er das an den Arduino
			cv2.putText(image_rgb, "intersection", (65, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 3)
			ser.write(b'S')
			print("Send: Skipped")
		linePos = int(x + w / 2 - 160)
		cv2.putText(image_rgb, str(linePos),(linePos + 140, 70), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 106, 255), 2)
		#cv2.line(image_rgb, (linePos + 160, 80), (linePos + 160, 160), (255, 0, 0),2)
		#cv2.line(image_rgb, (0, 110), (319, 110), (255, 0, 0), 2)
		lastLinePos = linePos
	contours_right = False
	contours_left = False	
	if(len(contours_grn) > 0):
		if(grn_counter <= 0):
			grn_counter = 10
		else:
			if(grn_counter == 1):
				left = 0
				right = 0
				d = False
				s = 0
				for c in grn_list:
					if(c == "L"):
						left = left + 1
						print("L")
					elif(c == "R"):
						right = right + 1
						print("R")
					elif(c == "D"):
						d = True
						print("D")
					elif(c == "S"):
						s = s + 1
						print("S")
				if(d): #hat doppel Gruen erkannt
					ser.write(b'D') 
					print("deadend") 
					print("Send: D")
					#delay(1)
				elif(s >= 6):
					#ser.write(b'S')
					#delay(0.2)
					print("Send: S")
				else:
					if(left > right):
						ser.write(b'L')
						print("Send: L")
						#delay(0.5)
					elif(right > left):
						ser.write(b'R')
						print("Send: R")
						#delay(0.5)
				grn_counter = 0
				grn_list.clear()
				#print("List cleared!")
				# for c in grn_list:
				# 	print(c)
		check = True
		for i in range(len(contours_blk)):
				b = cv2.boundingRect(contours_blk[index])
				x, y, w, h = b
				if(w > 1000):
					grn_list.append("S")
					check = False
		if(check):
			for i in range(len(contours_grn)):
				b = cv2.boundingRect(contours_grn[i])
				x, y, w, h = b
				cv2.rectangle(image_rgb, (x, y + CUT_GRN[2] + CUT_GRN[0]), (x + w, y + h + CUT_GRN[2] + CUT_GRN[0]), (0, 255, 0), 3) #rechteck um grüne Konturen, +CUT_GRN[2]??????
				a = x + w / 2 - 160
				if(a < linePos):
					contours_left = True
				elif(a > linePos):
					contours_right = True
	else:
		if(grn_counter > 0):
			print("abort")
			grn_counter = 0
			grn_list = []
	if(contours_left and contours_right):
		for i in range(len(contours_grn)):
			b = cv2.boundingRect(contours_grn[i])
			x, y, w, h = b
			#cv2.rectangle(image_rgb, (x, y + CUT_GRN[2]), (x + w, y + h + CUT_GRN[2]), (0, 255, 0), 3)
			a = x + w/2 - 160
			if(a < linePos):
				contours_left = True
			elif(a > linePos):
				contours_right = True
		if(contours_left and contours_right):
			grn_list.append("D")
	elif(contours_left):
		for i in range(len(contours_grn)):
			b = cv2.boundingRect(contours_grn[i])
			x, y, w, h = b
			#cv2.rectangle(image_rgb, (x, y + CUT[3]), (x + w, y + h + CUT[3]), (0, 255, 0), 3)
			a = x + w/2 - 160
			if(a < linePos):
				contours_left = True
			elif(a > linePos):
				contours_right = True
		if(contours_left):
			grn_list.append("L")
			if(grn_counter == 7):
				grn_list.append("L")
	elif(contours_right):
		for i in range(len(contours_grn)):
			b = cv2.boundingRect(contours_grn[i])
			x, y, w, h = b
			#cv2.rectangle(image_rgb, (x, y + CUT[3]), (x + w, y + h + CUT[3]), (0, 255, 0), 3)
			a = x + w/2 - 160
			if(a < linePos):
				contours_left = True
			elif(a > linePos):
				contours_right = True
		if(contours_right):
			grn_list.append("R")
			if(grn_counter == 7):
				grn_list.append("R")
	else:
		value = str(linePos).encode()
		value = int(float(value))

		if value == 0: #value == 0 -> Roboter befindet sich in einer Luecke, weil er keine Linie finden kann
			print(gapcounter)
			gapcounter = gapcounter + 1
			# if gapcounter >= 5:
			# 	#Luecke erkannt
			# 	gapcounter = 0
			# 	print("Luecke erkannt")
			# 	if LinePosLastLoop[7] > 10: #vor 6 Loop Durchlaeufen war str(linePos).encode() > 0, weshalb sich der Roboter jetzt ein Stueck nach rechts drehen muss, um die Luecke besser zu ueberfahren
			# 		print("gapR -> nach rechts drehen...")
			# 		delay(5)
			# 		#ser.write(b'gapR')
			# 		delay(0.05)
			# 	elif LinePosLastLoop[7] < -10: #vor 6 Loop Durchlaeufen war str(linePos).encode() < 0, weshalb sich der Roboter jetzt ein Stueck nach links drehen muss, um die Luecke besser zu ueberfahren
			# 		print("gapL -> nach links drehen...")
			# 		delay(5)
			# 		#ser.write(b'gapL')
			# 		delay(0.05)
			# 	elif LinePosLastLoop[7] == 0:
			# 		print("LinePosLastLoop[7] ist leider 0")
			# 	else: #fahre ein Stückchen gerade aus
			# 		pass
		else:
			gapcounter = 0
			ser.write(str(linePos / 10).encode()) 

	if(grn_counter > 0):
		grn_counter = grn_counter - 1
	framesTotal = framesTotal + 1


	rawCapture.truncate(0)
	DEBUG()
	# wird gebraucht, um bei Luecke (-> gapcounter >= Schwellwert) ein Stueck nach rechts oder links zu korrigieren
	LinePosLastLoop[7] = LinePosLastLoop[6]
	LinePosLastLoop[6] = LinePosLastLoop[5] 
	LinePosLastLoop[5] = LinePosLastLoop[4] 
	LinePosLastLoop[4] = LinePosLastLoop[3] 
	LinePosLastLoop[3] = LinePosLastLoop[2] #speichert den vor-vor-vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[2] = LinePosLastLoop[1] #speichert den vor-vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[1] = LinePosLastLoop[0] #speichert den vor-vorletzten str(linePos).encode() in Array
	LinePosLastLoop[0] = value #speichert den vorletzten str(linePos).encode() in Array


	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):

		print("Avg. FPS:", int(framesTotal / (time.time() - startTime - timeWaitet))) #sendet durchsch. Bilder pro Sekunde (FPS)
		camera.close()
		exit()