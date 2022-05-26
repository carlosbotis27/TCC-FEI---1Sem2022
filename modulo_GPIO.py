#importa bibliotecas
#import numpy as np
import RPi.GPIO as GPIO
import sys
import time
import matplotlib.pyplot as plt
import shutil

#função para definir a modelo de numeração para BOARD
def iniciar_board():
    """Função utilizada para iniciar o GPIO do raspberry"""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

#função para definir os pinos como saída
def definir_pinos_como_saida(numero_pino):
    """Função utilizada para definir determinada saída como OUT,\n
    :numero_pino = numeração da porta (BOARD)"""
    GPIO.setup(numero_pino, GPIO.OUT)

#função para escrever no pino
def definir_estado_porta(numero_pino, estado_porta):
    """Função utilizada para definir o estado das portas,\n
    como HIGH ou com LOW\n
    :numero_pino = numero do pino (BOARD)
    :estado_porta = iniciar_board(HIGH), iniciar_board(LOW)"""
    GPIO.output(numero_pino, estado_porta)

iniciar_board()

motor_direira = int(38)
motor_esquerda = int(40)
raspbery = int(7)

ligar = GPIO.HIGH
desligar = GPIO.LOW

definir_pinos_como_saida(motor_direira)
definir_pinos_como_saida(motor_esquerda)
definir_pinos_como_saida(raspbery)

definir_estado_porta(motor_direira,desligar)
definir_estado_porta(motor_esquerda,desligar)
definir_estado_porta(raspbery,ligar)

def ligar_motores_direita():
    """Função para ligar os motores da pulseira direita"""
    definir_estado_porta(motor_direira,ligar)
    
def ligar_motores_esquerda():
    """Função para ligar os motores da pulseira esquerda"""
    definir_estado_porta(motor_esquerda,ligar)
    
def desligar_motores():
    """Função para desligar os motores"""
    definir_estado_porta(motor_direira,desligar)
    definir_estado_porta(motor_esquerda,desligar)

def estado_atencao():
    """Função será ligada quando o objeto entrar dentro do limite de segurança"""
    for i in range(5):
        time.sleep(1)
        ligar_motores()
        time.sleep(1)
        desligar_motores()

def estado_critico():
    """Função será ligada quando o objeto estiver ultrapassado o limite de segurança e representa um perigo eminente"""
    for i in range(10):
        time.sleep(0.1)
        ligar_motores()
        time.sleep(0.1)
        desligar_motores()
        time.sleep(0.1)
        ligar_motores()
        time.sleep(0.05)
        desligar_motores()
        time.sleep(0.01)
        ligar_motores()
        time.sleep(1)
        
desligar_motores()
while(1):

    for i in range(100):
        if (i % 2 ==0):
            print("Dentro do aceitável")
            estado_atencao()
        else:
            print("Fora do aceitável")
            estado_critico()

        definir_estado_porta(motor_direira, desligar)
        definir_estado_porta(motor_esquerda, desligar)
    exit(0)

GPIO.cleanup()