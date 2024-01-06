
import numpy as np
import matplotlib.pyplot as plt


def histogramme(data, intervalle=None, nombre_de_cellules=10):
    if intervalle is None:
        intervalle = (min(data), max(data))
    bins = np.linspace(intervalle[0], intervalle[1], nombre_de_cellules + 1)
    histo = np.zeros(nombre_de_cellules)
    mode = (bins[:-1] + bins[1:]) / 2

    for d in data:
        for i in range(nombre_de_cellules):
            if bins[i] <= d < bins[i + 1]:
                histo[i] += 1
                break
    
    return histo, mode

# 示例数据
data = np.random.randn(100)  # 随机生成数据
histo, mode = histogramme(data)


def afficher_histogramme(histo, mode):
    plt.bar(mode, histo, width=mode[1] - mode[0])
    plt.show()

afficher_histogramme(histo, mode)
