import RPi.GPIO as GPIO
from time import sleep
porta=[33,35,36,37,38] # Portas da GPIO 
VEL=0.007
GPIO.setmode(GPIO.BOARD)
pwm = []
for p in porta:
    GPIO.setup(p, GPIO.OUT)
    pwm.append(GPIO.PWM(p, 50)) #pwm2=GPIO.PWM(PORTA_B, 50)

dutyCicle = []
f=open("motores.txt","r")
linha = f.readline()
for n in linha.split(';'):
    print(n)
    dutyCicle.append(float(n)) # inicia os dutys com o arquivo
f.close()

for p, d in zip(pwm, dutyCicle):
    p.start(d)

def move(id, angulo, vel=VEL):
    duty = angulo * 1.0 / 18 + 2
    GPIO.output(porta[id], True)
    origem = dutyCicle[id]
    destino = duty
    if(origem<destino):
        while origem<destino:
            origem+=0.01
            pwm[id].ChangeDutyCycle(origem)
            sleep(vel)
    else:
        while origem>destino:
            origem-=0.01
            pwm[id].ChangeDutyCycle(origem)
            sleep(vel)
    dutyCicle[id]=duty # atualiza o dutyCicle atual
    GPIO.output(porta[id], False)
    pwm[id].ChangeDutyCycle(0)

def demo():
    sleep(0.1)
    for i in range(5):
        move(i,45)
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
# Início do código
#
move(2,80)
move(4,100)
move(3,10)
demo()
#
# Final
#
for p in pwm:
    p.stop()
GPIO.cleanup()
# salvando posições em disco
f=open("motores.txt","w")
for i, d in enumerate(dutyCicle):
    f.write(str(d))
    if(i<4):
        f.write(";")
#f.seek(0) # vai para inicio do arquivo
f.close()
print("Fim!")
