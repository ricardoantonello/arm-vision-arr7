# E-mail: ricardo@antonello.com.br

# imports para camera
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
print(cv2.__version__)
import numpy as np
#from matplotlib import pyplot as plt
#import os
#import sys
#print(sys.executable)

# Imports para motores
import RPi.GPIO as GPIO
from time import sleep
#import readchar

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))
 


#Inicializa variáveis para motores
porta=[33,35,36,37,38] # Portas da GPIO 
VEL=0.005

#Inicia portas GPIO
GPIO.setmode(GPIO.BOARD)
pwm = []
for p in porta:
    GPIO.setup(p, GPIO.OUT)
    pwm.append(GPIO.PWM(p, 50)) # 50 hz

# Carrega dutyCicles do disco
dutyCicle_atual = []
f=open("motores_atual.txt","r")
linha = f.readline()
for n in linha.split(';'):
    print(n)
    dutyCicle_atual.append(float(n)) # inicia os dutys com o arquivo
f.close()
dutyCicle_gravado = []
f=open("motores_gravado.txt","r")
linha = f.readline()
for n in linha.split(';'):
    print(n)
    dutyCicle_gravado.append(float(n)) # inicia os dutys com o arquivo
f.close()


#print('::: Start pwm')
#for p, d in zip(pwm, dutyCicle_atual):
#        p.start(d)
#        #print(p, d)
#print('::: Start pwm FIM')
    
def angulo2dutycicle(a):
    return a / 18.0 + 2.0

def dutyCicle2angulo(d):
    return (d - 2.0) * 18.0

def move(motor_id, angulo, absoluto=True, vel=VEL):
    dn = angulo2dutycicle(angulo) # novo
    da = dutyCicle_atual[motor_id] # atual
    if not absoluto: # entao soma o destino na posição atual
        dn = da+dn-2
        #print('destino mudou para', dn)

    pwm[motor_id].start(da)
    GPIO.output(porta[motor_id], True)

    if(da<dn):
        while da<=dn:
            da+=0.01
            pwm[motor_id].ChangeDutyCycle(da)
            sleep(VEL)
            #print(da, dn)
    else:
        while da>=dn:
            da-=0.01
            pwm[motor_id].ChangeDutyCycle(da)
            sleep(VEL)
            #print(da, dn)
    GPIO.output(porta[motor_id], False)
    pwm[motor_id].ChangeDutyCycle(0)
    #pwm[motor_id].stop() # nao pode dar o stop.. senao pira...
    dutyCicle_atual[motor_id]=dn # atualiza o dutyCicle atual


def gravar_posicao():
    for i in range(5):
        dutyCicle_gravado[i]=dutyCicle_atual[i]
    print('Posicao gravada com sucesso')

def move_posicao_gravada():
    print('Seguindo para posicao gravada')
    for i in range(5): # para cada motor
        print('Movendo motor', i, 'para posicao', dutyCicle_gravado[i])
        move(i, dutyCicle2angulo(dutyCicle_gravado[i]))

def demo():
    sleep(0.1)
    for i in range(5):
        move(i,110) # para ficar de pé...
    sleep(1)
def abre_mao():
    move(0, 70)
    sleep(0.3)
def fecha_mao():
    move(0, 15)
    sleep(0.3)
def punho_vertical():
    move(1, 10) # punho vertical
    sleep(0.3)
def punho_horizontal():
    move(1, 97) # punho horizontal
    sleep(0.3)
#
# Início da captura de teclas
#
#print(angulo2dutycicle(10)) 
#print(dutycicle2angulo(2.555555))

passo = 10
distancia = 0
flag_movimento = False
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

        im = image.copy()
        #im = im[80:160,120:200]
        im0 = im[:300,100:220,0]
        im1 = im[:300,100:220,1]
        im2 = im[:300,100:220,2]
        #im[10:60,:,:] = (255,0,0)
        blur=cv2.GaussianBlur(im1, (7,7), 0)
        (T,binI)=cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY_INV)
        (T,binario)=cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)
        
        im=im1
        #borda_sup=10
        #im[borda_sup:120,40:70]=0
        #for i in range(0,00):
            #print(binI[borda_sup+40,55])
        #    if(binI[i+borda_sup, 55]>200):
        #        binI[i+borda_sup,40:70]=50


        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = 10
        params.filterByCircularity = True
        params.minCircularity = 0
        params.filterByConvexity = True
        params.minConvexity = 0

        det = cv2.SimpleBlobDetector_create(params)
        keypoints = det.detect(binario)
        for k in keypoints:
            cv2.circle(binI, (int(k.pt[0]),int(k.pt[1])), int(k.size)//2, 100, 2);
        

        if(len(keypoints)==2):
            d0=keypoints[0].pt[1]
            d1=keypoints[1].pt[1]
            distancia = d0-d1; # distancia onde d0 é sempre maior
            
            #print('x', x, 'z', z, 'distancia', distancia, 'dist_nova', dist_nova)
            #print('size kp0', keypoints[0].size)
            #print('size kp1', keypoints[1].size)
            if keypoints[0].size<keypoints[1].size:
                move(1, -passo, False)
            else:
                move(1, passo, False)
                            

        # show the frame
        #cv2.imshow("Frame0", im0)
        cv2.imshow("Frame1", im)
        #cv2.imshow("Frame2", im2)
        cv2.imshow("Blur", blur)
        cv2.imshow("Binario", binI)
        
        c = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if c == ord('q'):
            break
        elif c==ord('a'):
            move(0, passo, False)
        elif c==ord('z'):
            move(0, -passo, False)
        elif c==ord('s'):
            move(1, passo, False)
        elif c==ord('x'):
            move(1, -passo, False)
        elif c==ord('d'):
            move(2, passo, False)
        elif c==ord('c'):
            move(2, -passo, False)
        elif c==ord('f'):
            move(3, passo, False)
        elif c==ord('v'):
            move(3, -passo, False)
        elif c==ord('g'):
            move(4, passo, False)
        elif c==ord('b'):
            move(4, -passo, False)
        elif c==ord('p'):
            gravar_posicao()
        elif c==ord('i'):
            move_posicao_gravada()
        elif c==ord('o'):
            demo()
            
    
        
#demo()

#
# Final
#
print('::: Stop pwm')
for p in pwm:
    p.ChangeDutyCycle(0)
    p.stop()
print('::: Stop pwm FIM')
GPIO.cleanup()
print('::: Salvando arquivo...')
# salvando posições em disco
f=open("motores_atual.txt","w")
for i, d in enumerate(dutyCicle_atual):
    f.write(str(d))
    if(i<4):
        f.write(";")
print('Posicao Atual: ', dutyCicle_atual)
f=open("motores_gravado.txt","w")
for i, d in enumerate(dutyCicle_gravado):
    f.write(str(d))
    if(i<4):
        f.write(";")
print('Posicao Gravada: ', dutyCicle_gravado)
#f.seek(0) # vai para inicio do arquivo
f.close()
print("Fim!")

