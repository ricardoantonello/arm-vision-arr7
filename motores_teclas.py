import RPi.GPIO as GPIO
from time import sleep
import readchar

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
while True:
    c=readchar.readchar()
    if c=='q':
        break
    elif c=='a':
        move(0, passo, False)
    elif c=='z':
        move(0, -passo, False)
    elif c=='s':
        move(1, passo, False)
    elif c=='x':
        move(1, -passo, False)
    elif c=='d':
        move(2, passo, False)
    elif c=='c':
        move(2, -passo, False)
    elif c=='f':
        move(3, passo, False)
    elif c=='v':
        move(3, -passo, False)
    elif c=='g':
        move(4, passo, False)
    elif c=='b':
        move(4, -passo, False)
    elif c=='p':
        gravar_posicao()
    elif c=='i':
        move_posicao_gravada()
        
    
        
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

