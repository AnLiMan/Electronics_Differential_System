#----Библиотеки---
import matplotlib.pyplot as plt
from math import sin
from random import gauss
from scipy import signal
import numpy as np

# ----Настройки---
points = 500 #Количество точек для расчёта
resolution = 0.05 #Шаг расчёта
lower_Gauss_threshold = 0.0 # Нижний порог для генерации шума
upperer_Gauss_threshold = 0.1 # Верхний порог для генерации шума
saw_offset = 1.0 # Смещение центра пилы (от 0 до 1)
saw_frequency = 0.5 # Частота пилы
graphs_output = True

#Списки значений функций
time = []
sinus = []
square = []
triangle = []
noize = []
sinus_noize = []
square_noize = []
triangle_noize = []

#----Функции----

#1. Функция отрисовки графиков
def draw_figure(title, xlabel, ylabel, x, y):
    plt.figure(figsize=(10, 6))
    plt.xlim(0, points * resolution)
    plt.grid()
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.plot(x, y, color = 'r')
    plt.hlines(0, 0, points*resolution)
    plt.show()

#2. Функция генерации шума
def noised():
    for i in range(0, points):
        noize.append(gauss(lower_Gauss_threshold, upperer_Gauss_threshold))
        sinus_noize.append(sinus[i] + noize[i])
        square_noize.append(square[i] + noize[i])
        triangle_noize.append(triangle[i] + noize[i])


#Расчёт времени
for i in range(0, points):
    time.append(resolution * i)

#Расчёт синуса
for i in range(points):
    sinus.append(sin(time[i]))

#Расчёт квадрата
for i in range(points):
    if (sinus[i] > 0):
        sq = 1
    else:
        sq = -1
    square.append(sq)

#Расчёт треугольного сигнала
for i in range(points):
    tr = signal.sawtooth(saw_frequency * np.pi * time[i], saw_offset)
    triangle.append(tr)

#-----Вывод графиков----
noised() # Пошумим

if (graphs_output == True):
    draw_figure("Синус", "Время, c", "Y", time, sinus)
    draw_figure("Квадратный сигнал", "Время, c", "Y", time, square)
    draw_figure("Треугольник волнообразный", "Время, c", "Y", time, triangle)
    draw_figure("Сгенерированный белый шум", "Время, c", "Y", time, noize)
    draw_figure("Синус зашумленный", "Время, c", "Y", time, sinus_noize)
    draw_figure("Квадратный сигнал зашумленный", "Время, c", "Y", time, square_noize)
    draw_figure("Треугольник волнообразный зашумленный", "Время, c", "Y", time, triangle_noize)
else:
    print("Для отображения графиков изменить значение переменной 'graphs_output' в скрипте 'signal_and_noize'")
