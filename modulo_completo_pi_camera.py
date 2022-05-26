import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
from datetime import date
import argparse
import RPi.GPIO as GPIO
import sys
import os
import os.path
import time
import matplotlib.pyplot as plt
import shutil


#Cria pasta para registro de segurança
if not os.path.exists('/home/pi/Documents/registros'):
    os.makedirs('/home/pi/Documents/registros')

# Parâmetros auxiliares iniciais (fonte para textos e cores em BGR) 
fonte_textos = cv2.FONT_HERSHEY_SIMPLEX
tamanho_texto = 0.4
cor_texto = (255,255,255)
cor_grade = (255,255,255)
cor_rosto = (255,255,255)
cor_linha = (96,32,0)
cor_linha_max = (80,176,0)
cor_linha_med =(0,255,255)
cor_alerta = (0,0,255)
cor_ret = (0,0,0)
hoje = str(date.today())

#variaveis auxiliares GaussianBlur
tam_kernel_gauss = (5,5)
desvio_padrao_gauus = 0

#variavel auxiliar filtro morphology
kernel_morphology = np.ones((20,20),np.uint8)

#variaveis auxiliares de posição dos textos
posicao_ret_aux = (5,5)
posicao_data = (10,20)
posicao_fps = (10,40)
posicao_distancia = (10,60)
largura_ret_aux = 8
escala_fonte = 1
posicao_dist_medida = (70, 60)

#variaveis auxiliares filtro de borda (canny)
limiar_max = 40
limiar_min = 20
razao = 2

#variaveis auxiliares para o detectmultiscale
fator_escala = 1.1
n_vizinhos = 1
lin_retangulo = 1
fator_escala_aux = 1.01
n_vizinhos_aux = 3

#Faz a leitura da câmera
# =============================================================================
# vs = cv2.VideoCapture(0)
# vs.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# vs.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# 
# =============================================================================
width, Height = 640, 480

#Inicializa a Picamera
camera = PiCamera()
camera.resolution = (width, Height)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(width, Height))

#Definir os classificadores
#classificador_carro = cv2.CascadeClassifier('/home/pi/Documents/classificadores/cars.xml')
classificador_carro = cv2.CascadeClassifier('/home/pi/Documents/dados/custom haar cascade/classifier/cascade.xml')
classificador_corpo = cv2.CascadeClassifier('/home/pi/Documents/classificadores/haarcascade_fullbody.xml')
classificador_rosto = cv2.CascadeClassifier('/home/pi/Documents/classificadores/haarcascade_frontalface_alt.xml')

#Captura o tamanho do video (x,y)
# =============================================================================
# x_do_video = int(vs.get(cv2.CAP_PROP_FRAME_WIDTH))
# y_do_video = int(vs.get(cv2.CAP_PROP_FRAME_HEIGHT))
# =============================================================================

x_do_video = int(640)
y_do_video = int(480)


# =============================================================================
# #captura do FPS
# fps_video = str(vs.get(cv2.CAP_PROP_FPS))
# fps_video =  fps_video + " FPS"
# =============================================================================

#captura do FPS
fps_video = str(30)
fps_video =  fps_video + " FPS"

#parametros de cálculos auxiliares
objeto_area = 0
objeto_larura = 0
objeto_altura = 0

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
    
def estado_atencao_direita():
    """Função será ligada quando o objeto entrar dentro do limite de segurança vindo do lado direito"""
    
    for i in range(5):
        time.sleep(1)
        ligar_motores_direita()
        time.sleep(1)
        desligar_motores()

def estado_critico_direita():
    """Função será ligada quando o objeto estiver ultrapassado\n o limite de segurança e representa um perigo eminente vindo do lado direito"""
    
    for i in range(10):
        time.sleep(0.01)
        ligar_motores_direita()

def ligar_motores_esquerda():
    """Função para ligar os motores da pulseira esquerda"""
    
    definir_estado_porta(motor_esquerda,ligar)

def estado_atencao_esquerda():
    """Função será ligada quando o objeto entrar dentro do limite de segurança vindo do lado esquerdo"""
    
    for i in range(5):
        time.sleep(1)
        ligar_motores_esquerda()
        time.sleep(1)
        desligar_motores()

def estado_critico_esquerda():
    """Função será ligada quando o objeto estiver ultrapassado\n o limite de segurança e representa um perigo eminente vindo do lado esquerdo"""
    
    for i in range(10):
        time.sleep(0.01)
        ligar_motores_esquerda()

def desligar_motores():
    """Função para desligar os motores"""
    
    definir_estado_porta(motor_direira,desligar)
    definir_estado_porta(motor_esquerda,desligar)

#propriedades da camera/video
# =============================================================================
# def propriedades_camera():
#     """Funcao para imprimir as propriedades da camera"""
#     
#     print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(vs.get(cv2.CAP_PROP_FRAME_WIDTH)))
#     print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(vs.get(cv2.CAP_PROP_FRAME_HEIGHT)))
#     print("CAP_PROP_FPS : '{}'".format(vs.get(cv2.CAP_PROP_FPS)))
#     print("CAP_PROP_POS_MSEC : '{}'".format(vs.get(cv2.CAP_PROP_POS_MSEC)))
#     print("CAP_PROP_FRAME_COUNT  : '{}'".format(vs.get(cv2.CAP_PROP_FRAME_COUNT)))
#     print("CAP_PROP_BRIGHTNESS : '{}'".format(vs.get(cv2.CAP_PROP_BRIGHTNESS)))
#     print("CAP_PROP_CONTRAST : '{}'".format(vs.get(cv2.CAP_PROP_CONTRAST)))
#     print("CAP_PROP_SATURATION : '{}'".format(vs.get(cv2.CAP_PROP_SATURATION)))
#     print("CAP_PROP_HUE : '{}'".format(vs.get(cv2.CAP_PROP_HUE)))
#     print("CAP_PROP_GAIN  : '{}'".format(vs.get(cv2.CAP_PROP_GAIN)))
#     print("CAP_PROP_CONVERT_RGB : '{}'".format(vs.get(cv2.CAP_PROP_CONVERT_RGB)))
# =============================================================================

#parametros para localizar a distância focal
#distância(em cm) da câmera ao objeto durante a captura da imagem de referência
distancia_conhecida = 50.00
#largura de um objeto do mundo real (utilizar frente do carro)
largura_real = 14.00

def distancia_foco(distancia_conhecida,largura_real,largura_pixels):
    """Funcao utilizada para capturar a distancia focal, onde:
    \n :distancia_conhecida=distância da câmera ao objeto durante a captura da imagem de referência
    \n :largura_real = largura de um objeto do mundo real (utilizar frente do carro)
    \n :largura_pixels = largura do retangulo que ficará no frame (pixels)(w)"""
    
    distancia_focal = (largura_pixels*distancia_conhecida)/largura_real
    
    return distancia_focal

def medir_distancia(retorno_dist_focal,largura_real_ref,largura_pixels_ref):
    """Função utilizada para medir a distancia da lente até o objeto
    \n :retorno_dist_focal = retorno da função distancia_foco
    \n :largura_real_ref = largura de um objeto do mundo real (utilizar frente do carro)
    \n :largura_pixels_ref = largura do retangulo que ficará no frame (pixels)(w)"
    \n :todas as distancias sao dadas em centimetros"""
    
    distancia_real = (largura_real_ref*retorno_dist_focal)/largura_pixels_ref
    
    return distancia_real

def dados_objeto(frame):
    """Função utilizada para retornar a largura em pixels do objeto
    /n :frame = imagem desejada"""
    
    #largura do retangulo que ficará no frame (pixels)
    largura_pixels = 0
    
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    faces = classificador_carro.detectMultiScale(frame,fator_escala_aux,n_vizinhos_aux)
    
    for (x,y,w,h) in faces:
        
        cv2.rectangle(frame, (x,y), ((x+w),(y+h)),cor_rosto, lin_retangulo)
        
        largura_pixels = w
    
    return largura_pixels

#carrega a imagem de referência
img_referencia = cv2.imread("/home/pi/Documents/dados/img_ref/img_46.jpg")

#encontrar a larura em pixels do objeto
largura_obj_ref = dados_objeto(img_referencia)

print("Largura ref : {}".format(largura_obj_ref))

#encontrar a distancia focal da imagem de referencia
distancia_focal_ref = distancia_foco(distancia_conhecida, largura_real, largura_obj_ref)

print("Distancia focal ref: {}".format(distancia_focal_ref))

# Funções auxiliares para o tratamento da imagem
def aplicar_filtros_imagem(frame):
    
    """Utilize esta função para tratar a imagem desejada,\n
    primeiro a imagem será convertida para uma escala de cinza (cvtColor),\n
    o segundo passo é aplicar um filtro passa baixa para remover os ruidos da imagem (GaussianBlur),\n
    depois é aplicado uma metodologia para encontrar o limiar adaptativo (adaptiveThreshold)\n
    e posteriormente é aplicado um filtro para detectar as bordas da imagem"""
    #converte para escala de cinza
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #preencher os vazios
    frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel_morphology)
    #equaliza os tons de cinza
    frame = cv2.equalizeHist(frame)
    #aplica o filtro passa baixa - desfoca a imagem para remover os ruídos, GaussianBlur(imagem,matriz que representa o tamanho do kernel,desvio padrão do kernel gaussiano)
    frame = cv2.GaussianBlur(frame, tam_kernel_gauss, desvio_padrao_gauus)
    #filtro para encontrar o limiar adaptativo,adaptiveThreshold(imagem,valor máximo=255, ADAPTIVE_THRESH_MEAN_C ou ADAPTIVE_THRESH_GAUSSIAN_C,tipo de limite,vizinhos,constante) 
    frame = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    #Filtro para detecar as bordas da imagem e também remover os ruidos. Canny(imagem, limite inferior no limite de histerese, limite superior no limite de histerese)
    frame = cv2.Canny(frame, limiar_min, int(limiar_max*razao))

    return frame
    
def inserir_grade(frame):
    """Utilize esta função para inserir a grade auxilia na imagem,\n
    esta grade será utilizada para auxiliar na localização do objeto,\n
    :frame = imagem que deseja inserir as linhas"""
    cont = 0
    x_aux = int(x_do_video/4)
    cstx = x_aux
    y_aux = int(y_do_video/4)
    csty = y_aux
    
    for cont in range(1,5):
        cv2.line(frame,(x_aux,0),(x_aux,y_do_video),cor_grade,1)
        cv2.line(frame,(0,y_aux),(x_do_video,y_aux),cor_grade,1)
        x_aux = x_aux + cstx
        y_aux = y_aux + csty
        
    return frame

def inserir_linhas(frame):
    """Utilize esta função para inserir as linhas auxiliares na imagem,\n 
    estas linhas serão utilizadas para medir a distância em que o objeto se encontrar,\n
    :frame = imagem que deseja inserir as linhas"""
    
    x_aux = int(x_do_video/4)
    y_aux = int(y_do_video/4)

    #definição do limite aceitável
    #linha lado esquerdo
    cv2.line(frame,(int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),(x_aux,y_aux),cor_linha_max,cv2.LINE_4)
    #linha lado direito
    cv2.line(frame,(x_do_video-int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),(x_do_video-x_aux,y_aux),cor_linha_max,cv2.LINE_4)
    #linha horizontal
    cv2.line(frame,(x_aux,y_aux),(x_do_video-x_aux,y_aux),cor_linha_max,cv2.LINE_4)

    #definição do limite crítico
    #linha lado esquerdo
    cv2.line(frame,(int(x_aux/2),y_do_video),(int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),cor_alerta,cv2.LINE_4)
    #linha lado direito
    cv2.line(frame,(x_do_video-int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),(x_do_video-int((x_aux/2)),y_do_video),cor_alerta,cv2.LINE_4)
    #linha horizontal
    cv2.line(frame,(int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),(x_do_video-int(x_aux-x_aux/4),y_do_video-2*y_aux+int(y_aux/2)),cor_alerta,cv2.LINE_4)
    
    return frame

def inserir_textos_aux(frame):
    """Utilize esta função para inserir os textos auxiliares no video,\n
    :frame = imagem que deseja inserir as linhas"""
    
    x_aux = int(x_do_video/4)
    y_aux = int(y_do_video/4)

    #texto do limite aceitável
    cv2.putText(frame, "Distancia aceitavel",(x_aux+int(x_aux/12),y_aux+int(y_aux/6)), fonte_textos, tamanho_texto, cor_texto,escala_fonte,cv2.LINE_AA)

    #texto do limite crítico
    cv2.putText(frame, "Distancia critica",(x_aux+int(x_aux/12),y_do_video-y_aux+int(y_aux/6)),fonte_textos,tamanho_texto, cor_texto,escala_fonte,cv2.LINE_AA)
    
    #insere o retangulo que servirá como apoio
    cv2.rectangle(frame, posicao_ret_aux, ((int(x_do_video/4))-int(int(x_do_video/4)/8),((int(y_do_video/4))-int(int(y_do_video/4)/6))), cor_ret,-1,largura_ret_aux)
    #insere a data de hoje no video
    cv2.putText(frame, hoje,posicao_data, fonte_textos, tamanho_texto, cor_texto,escala_fonte,cv2.LINE_AA)
    #insere o FPS do video
    cv2.putText(frame, fps_video,posicao_fps, fonte_textos, tamanho_texto, cor_texto,escala_fonte,cv2.LINE_AA)
    #insere o texto Distância:
    cv2.putText(frame, "Distancia: ",posicao_distancia, fonte_textos, tamanho_texto, cor_texto,escala_fonte,cv2.LINE_AA)
    
    return frame

def preparar_frame(frame):
	"""Utilize esta função para preparar a imagem para ser analizada,\n
    : frame = imagem que deseja tratar"""
    
	frame = aplicar_filtros_imagem(frame)
	frame = inserir_grade(frame)
	frame = inserir_linhas(frame)
	frame = inserir_textos_aux(frame)
	
	return frame

def lado_objeto(x):
    """Função para identificar em qual lado o objeto identificado se encontra\n
    : x = posição em x do objeto identificado"""
    
    ref_esq = int(x_do_video/2)
    
    if (x - ref_esq) > 0:
        lado_objeto = "Lado esquerdo"
    else:
        lado_objeto = "Lado direito"
        
    return lado_objeto

def aproximacao_objeto(y,h):
    """Função para verificar auxiliar na verificação se o objeto está longe\n
    : y = altura em y do objeto identificado"""
    
    y_seguro = int(y_do_video/4)
    y_critico = int(y_do_video/2)+int((y_do_video/4)/2)
    var_aux = y + h

    if (var_aux <= y_seguro and var_aux < y_critico):
        loc_objeto = "Distancia segura"
    elif (var_aux > y_seguro and var_aux <= y_critico):
        loc_objeto = "Distancia alerta"
    elif (var_aux >= y_critico):
        loc_objeto = "Distancia critica"
        
    return loc_objeto

def verificar_objeto(lado_objeto,aproximacao_objeto,distancia):
    """Funcao para verificar se o objeto representa perigo\n
    : aproximacao_objeto = indica se o objeto está a uma distancia segura, de alerta ou critica\n
    : distancia = medicao da distancia em que o objeto se encontra\n"""
    
    dist_segura = 80.00
    dist_critica = 40.00
    
    if distancia > dist_segura and aproximacao_objeto == "Distancia segura":
        print("Objeto não representa perigo")
        desligar_motores()
        pass
    elif distancia <= dist_segura and aproximacao_objeto == "Distancia alerta":
        print("Objeto está a uma distancia que pode representar perigo")
        
        if lado_objeto=="Lado esquerdo":
            estado_atencao_esquerda()
        elif lado_objeto=="Lado direito":
            estado_atencao_direita()
            
    elif distancia < dist_critica and aproximacao_objeto =="Distancia critica":
        print("Objeto está muito perto, risco a integridade física")
        cv2.imwrite("/home/pi/Documents/registros/img_obj_" + hoje + str(y+h) + ".jpg", frame)
        
        if lado_objeto=="Lado esquerdo":
            estado_critico_esquerda()
        elif lado_objeto=="Lado direito":
            estado_critico_direita()
            

cont_frame = 0
#while True:
for image_array in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
	frame = image_array.array
    
	cont_frame +=1

	if frame is None:
		break

	largura_pixels_ref = dados_objeto(frame)

	carros = classificador_carro.detectMultiScale(frame,fator_escala,n_vizinhos)

	frame = preparar_frame(frame)

	for (x,y,w,h) in carros:
		
		aux_lado = lado_objeto(x)
        
		aux_aprox = aproximacao_objeto(y,h)

	if largura_pixels_ref != 0:

		distancia = medir_distancia(distancia_focal_ref, largura_real, largura_pixels_ref)
        
		cv2.putText(frame, f" {round(distancia,2)} cm",posicao_dist_medida,fonte_textos,tamanho_texto, cor_texto,1,cv2.LINE_AA)
        
		verificar_objeto(aux_lado,aux_aprox,distancia)
	
	#mostra a imagem em tempo real
	cv2.imshow("Video em tempo real", frame)
	
	#sair do loop se pressionar q(quit)
	if cv2.waitKey(1) == ord("q"):
		print("Video interrompido")
		break

#fechar a camera
#vs.release()

#fechar o video aberto
cv2.destroyAllWindows()
 
parser = argparse.ArgumentParser(description='Sistema de auxilio a mobilidade de deficientes visuais')
parser.add_argument('--face_cascade', help='Path to face cascade.', default='data/haarcascades/haarcascade_frontalface_alt.xml')
parser.add_argument('--eyes_cascade', help='Path to eyes cascade.', default='data/haarcascades/haarcascade_eye_tree_eyeglasses.xml')
parser.add_argument('--camera', help='Camera divide number.', type=int, default=0)
args = parser.parse_args()

face_cascade_name = args.face_cascade
eyes_cascade_name = args.eyes_cascade
camera = args.camera

classificador_carro = cv2.CascadeClassifier()
#eyes_cascade = cv2.CascadeClassifier()

# =============================================================================
# #-- 1. Carregando os classificadores
# if not classificador_carro.load(cv2.samples.findFile(face_cascade_name)):
#     print('--(!)Erro ao carregar o classificador de corpo loading')
#     exit(0)
# 
# #-- 2. Leitura do video
# if not vs.isOpened(camera):
#     print("--(!) Verifique a entrada da câmera -- break")
#     exit(0)
#     
# while True:
#     ret, frame = vs.read()
#     if frame is None:
#         print('--(!) Sem imagem para tratar -- Break!')
#         break
# =============================================================================
