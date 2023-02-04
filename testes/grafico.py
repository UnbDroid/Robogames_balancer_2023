import csv
import matplotlib.pyplot as plt


iteration = []
potencia_antes = []
potencia_limitador = []
velocidade_referencia = []
velocidade = []
thetha1 = []
thetha2 = []
encoder1 =[]
encoder2 = []

def readFile():
  with open('output1.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
    for idx, row in enumerate(spamreader):
        iteration.append(idx)
        try:
            potencia_antes.append(float(row[1]))
            # potencia_limitador.append(float(row[1]))
            # velocidade_referencia.append(float(row[2]))
            # velocidade.append(float(row[3]))
            # thetha1.append(float(row[4]))
            # thetha2.append(float(row[5]))
            # encoder1.append(int(row[6]))
            # encoder2.append(int(row[7]))
        except:
            print('Error found')
  

def plotGraph():
    # print(potencia_antes)
    # print(len(potencia_antes))
    # print(len(iteration))
    # potencia_antes.append(0.1)
    # encoder1.append(1)
    # encoder1.append(1)
    # potencia_antes.append(0.1)
    # potencia_antes.append(1)
    # plt.plot(iteration, potencia_antes, label='potencia_antes')
    plt.plot(iteration, potencia_antes, label='velocidade')
    
    # plt.plot([1,2,3], [1, 2, 3])
    # plt.title('Potencia Antes')
    plt.xlabel('iteration')
    # plt.ylabel('potencia_antes')
    plt.show()

readFile()
plotGraph()

