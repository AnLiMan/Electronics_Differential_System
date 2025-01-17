#----Библиотеки---
import signal_and_noize
import matplotlib.pyplot as plt
import math

# ----Настройки---
averange_resolution = 3 #Количество точек выборки для среднего арифметического
running_average_k = 0.17 # Коэ-т фильтрации бегущего среднего от 0 до 1.0
err_measure = 0.1 #Примерный шум измерений (для фильтра Калмана)
q = 0.1 # Скорость изменения значений от 0.001 до 1.0 (для фильтра Калмана)

#---Переменные---
averange_list = [] #Список отфильтрованых значений для среднего арифметического
time_averange_list = [] #Список значений времени для среднего арифметического
median_filter_list = [] #Список отфильтрованых значений для медианного фильтра
time_median_list = [] #Список  значений времени для медианного фильтра
sinus_noize_modifited = [] #Список отфильтрованый значений для среднего арифметического
running_average_list = [] # Список отфильтрованых значений для фильтра бегущее среднее экспоненциальное
kalman_list = [] # Список отфильтрованых значений для фильтра Калмана

#----Функции---

# 1. Вывод 2-х графиков в одной координатной системе
def drawing_figures (title, xlabel, ylabel, x1, y1, x2, y2, color1 = "r", color2 = "g"):
    plt.figure(figsize = (10, 6))
    plt.xlim(0, signal_and_noize.points * signal_and_noize.resolution)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid()
    plt.hlines(0, 0, signal_and_noize.points * signal_and_noize.resolution)
    plt.plot(x1, y1, color = color1, label = "После фильтра")
    plt.plot(x2, y2, color = color2, label = "До фильтра")
    plt.legend()
    plt.show()


# 2.Среднее арифметическое (фильтр)
def average (x):
    sum = 0
    for i in range(0, len(x)):
        sum = sum + x[i]
        if ((i + 1) % averange_resolution == 0):
            averange_list.append(sum / averange_resolution)
            time_averange_list.append(signal_and_noize.time[i])
            sum = 0

#3. Бегущее среднее экспоненциальное с адаптивным коэффициентом фильтрации (фильтр)
def running_averange(x):
    fil_val = 0
    for i in range(0, len(x)):
        if math.fabs((x[i] - fil_val) > 1.5):
            k = 0.9
            fil_val += (x[i] - fil_val) * k
        else:
            fil_val += (x[i] - fil_val) * running_average_k
        running_average_list.append(fil_val)

#4. Медианный фильтр
def median_filter(x):
    for i in range(0, len(x)-2):
        middle = max(x[i], x[i + 2]) if (max(x[i], x[i + 1]) == max(x[i + 1], x[i + 2])) else max(x[i + 1], min(x[i], x[i + 2]))
        median_filter_list.append(middle)

#5. Фильтр Калмана (упрощённый)
def kalman_filter(x):
    kalman_gain = 0
    current_estimate = 0
    err_estimate = err_measure
    last_estimate = 0
    for i in range(0, len(x)):
        kalman_gain = err_estimate / (err_estimate + err_measure)
        current_estimate = last_estimate + kalman_gain * (x[i] - last_estimate)
        err_estimate = (1.0 - kalman_gain) * err_estimate + math.fabs(last_estimate - current_estimate) * q
        last_estimate = current_estimate
        kalman_list.append(current_estimate)

#---Проверка фильтров----

#1.Среднее арифметическое
average(signal_and_noize.sinus_noize)
drawing_figures("Фильтр 'Арифметическое среднее'", "Время", "Y", time_averange_list, averange_list,
                signal_and_noize.time, signal_and_noize.sinus_noize)
signal_and_noize.draw_figure("Отфильтрованые значения средним арифметическим", "Время", "Y", time_averange_list, averange_list)

#2. Бегущее среднее экспоненциальное с адаптивным коэффициентом фильтрации
running_averange(signal_and_noize.sinus_noize)
drawing_figures("Фильтр 'Бегущее среднее экспоненциальное'", "Время", "Y", signal_and_noize.time, running_average_list,
                signal_and_noize.time, signal_and_noize.sinus_noize)
signal_and_noize.draw_figure("Отфильтрованые значения бегущим средним экспоненциальным", "Время", "Y", signal_and_noize.time, running_average_list)

running_average_list = []
running_averange(signal_and_noize.square_noize)
drawing_figures("Фильтр 'Бегущее среднее экспоненциальное'", "Время", "Y", signal_and_noize.time, running_average_list,
                signal_and_noize.time, signal_and_noize.square_noize)
signal_and_noize.draw_figure("Отфильтрованые значения бегущим средним экспоненциальным", "Время", "Y", signal_and_noize.time, running_average_list)

running_average_list = []
running_averange(signal_and_noize.triangle_noize)
drawing_figures("Фильтр 'Бегущее среднее экспоненциальное'", "Время", "Y", signal_and_noize.time, running_average_list,
                signal_and_noize.time, signal_and_noize.triangle_noize)
signal_and_noize.draw_figure("Отфильтрованые значения бегущим средним экспоненциальным", "Время", "Y", signal_and_noize.time, running_average_list)

#3. Фильтр Калмана
kalman_filter(signal_and_noize.sinus_noize)
drawing_figures("Фильтр Калмана", "Время", "Y", signal_and_noize.time, kalman_list,
                signal_and_noize.time, signal_and_noize.sinus_noize)
signal_and_noize.draw_figure("Отфильтрованые значения фильтром Калмана", "Время", "Y", signal_and_noize.time, kalman_list)

#4.Медианный
median_filter(signal_and_noize.sinus_noize) # Считаем медиану
# Некоторое количество подгона, так как кол-во значений фильтра неодинаково с исходными значениями функции
time_median_list = signal_and_noize.time
time_median_list.pop(signal_and_noize.points-1)
time_median_list.pop(signal_and_noize.points-2)
sinus_noize_modifited = signal_and_noize.sinus_noize
sinus_noize_modifited.pop(signal_and_noize.points-1)
sinus_noize_modifited.pop(signal_and_noize.points-2)

drawing_figures("Медианный фильтр", "Время", "Y", time_median_list, median_filter_list,
                signal_and_noize.time, signal_and_noize.sinus_noize)
signal_and_noize.draw_figure("Отфильтрованые значения медианным фильтром", "Время", "Y", time_median_list, median_filter_list)