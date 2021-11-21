# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 12:46:51 2021

@author: Fabrizio Figari Llosa
código: 20190728
IMPORTANTE:
Hice el algoritmo de enrutamiento de estado de enlace con dijkastra como conversamos en asesoría profe
"""

from collections import defaultdict
import sys
import time
import os
import psutil

#Estructura para almacenar los vertices a procesar
class Heap(): 
    #Constructor de la estructura Heap
    def __init__(self):
        self.lista = [] #lista de nodos
        self.tamaño = 0 #cuantos nodos hay
        self.pos = []   #lista con las posiciones respectivas
    
    def CrearNodo(self,v,dist): #cada nodo es una lista con el numero del
        nodo = [v,dist]         #vertice y la distacia desde el nodo source
        return nodo             
    
    
    #Intercambia los nodos del heap para poder poder arreglar el Heap de 
    #valores minimos
    def intercambiarNodos(self,nodo1,nodo2):
        aux = self.lista[nodo1]             
        self.lista[nodo1] = self.lista[nodo2]
        self.lista[nodo2] = aux
        return      


    #Convertimos el heap en un heap de valores minimos
    #            1
    #          /  \
    #         2   3
    def ConvertirEnMinHeap(self, index): 
        minimo = index                  
        izq = 2 * index + 1             
        der = 2 * index + 2             
                                        
        if izq < self.tamaño and self.lista[izq][1] < self.lista[minimo][1]:
            minimo = izq
        if der < self.tamaño and self.lista[der][1] < self.lista[minimo][1]:
            minimo = der
            
        if minimo != index:
            #Se intercambian las posiciones de la lista de posiciones
            self.pos[self.lista[minimo][0]] = index
            self.pos[self.lista[index][0]] = minimo
            #Se intercambian los nodos del heap minimo
            self.intercambiarNodos(minimo,index)
            self.ConvertirEnMinHeap(minimo)
            
    #Funcion que permite verficar si el Heap está vacio 
    def isEmpty(self):
        if self.tamaño == 0:
            return True
        else:
            return False
        
    #Funcion para extraer el primer nodo minimo del Heap
    def extraerMinimo(self):  
        if self.isEmpty(): #si el heap está vacio se retorna null
            return
        
        # Guardamos el nodo a extraer
        nodoAExtraer = self.lista[0]
        # Reemplazamos el nodo raiz con el ultimo nodo
        ultimo = self.lista[self.tamaño-1]
        self.lista[0] = ultimo
        
        # Se actualiza la posicion del ultimo nodo en la lista de posiciones
        self.pos[ultimo[0]] = 0
        self.pos[nodoAExtraer[0]] = self.tamaño - 1
        
        #Se reduce el tamaño del Heap por el nodo extraido 
        self.tamaño -= 1
        #Se vuelve a ordenar el Heap en un Heap de valores minimos
        self.ConvertirEnMinHeap(0)
        
        return nodoAExtraer
        
    #Se van disminuyendo las distancias según se encuentren caminos más cortos
    # y se reordenan (operación basica de un heap)
    def disminuirDistanciaMin(self,v,distancia):
        
        #Se obtiene el indice del vertice v en el heap
        i = self.pos[v]
        #Se actualiza la distancia minima
        self.lista[i][1] = distancia
        
        #Se recorre mientras el Heap no esté ordeneado por valores minimos
        while i > 0 and self.lista[i][1] < self.lista[(int)((i - 1) // 2)][1]:
            
            #Si intercambia el nodo hijo de menor valor por el padre 
            self.pos[self.lista[i][0]] = (i-2) // 2
            self.pos[self.lista[(int)((i-2) // 2)][0]] = i
            self.intercambiarNodos(i, (int)((i-2) // 2))
            
            #Ahora el vertice se encuentra en el indice del ex-padre
            i = (int) ((i-2) // 2)
    
    #Funcion para verificar si un vertice está en la estructura Heap
    def estaEnElHEap(self,v):
        if self.pos[v] < self.tamaño:
            return True
        return False
    
#Se convierten los Link-state packet en vertices para del grafo (NO DIRIGIDO)
def LSP(grafo,router1,conexiones):
    for n in conexiones:
        grafo.añadirConexion2Routers(router1, n[0], n[1])
        grafo.aristas += 1
    return
    
    
#Mostrar las distancias minimas por nodo
def printRutas(dist, n , salto, raiz = "0"): 
        print(f"""→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→
↓\t\t\t↓\tDistancia\t\t↓\tSalto\t↓
↓\tRouter\t↓\tdesde el router\t↓\tPrevio\t↓
↓      \t\t↓\t"{raiz}" \t\t\t↓\t\t    ↓
→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→""")
        for i in range(n):
            if i < 10:
                print(f"↓\t  0{i}\t↓\t\t{dist[i]}\t\t\t↓\t  {salto[i]}\t\t↓")
            else:
                print(f"↓\t  {i}\t↓\t\t{dist[i]}\t\t\t↓\t  {salto[i]}\t\t↓")
        print("→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→")
class Graph():
        
    #Constructor que necesita el numero de nodos del grafo/topología: V
    def __init__(self,V):
            self.V = V 
            #Se inicializa el tamaño del grafo(V) con el parametro V del constructor 
            self.graph = defaultdict(list)
            #Se crea un diccionario de listas para almacenar los nodos y sus pesos 
            #respectivos
            self.aristas = 0;
    
    def añadirConexion2Routers(self,router1,router2,peso):
            
            nodoNuevo = [router2,peso]
            self.graph[router1].insert(0,nodoNuevo)
            #se inserta en la llave src en la posición 0 de la lista 
            # de nodos el nuevo nodo con el vertice adyacente y el
            #peso de la arista (lista de nodos{src:[[V1,peso],[V2,peso]]})
            
    #Funcion para aplicar Dijkstra al grafo y calcular todas las distancias
    #minimas del nodo raiz hacia los demás       
    def aplicaDijkstra(self, raiz):
            
            V = self.V #Se obtiene el atributo nro de vetices
            distancias = [] #Se crea una lista con las distancias para elegir la minima
            salto = [] #Se crea una lista con los últimos saltos en cada nodo
            minHeap = Heap()
            
            #Se inicializa la estructura de datos Heap con todos los vertices
            for v in range(V):
                distancias.append(sys.maxsize)
                minHeap.lista.append(minHeap.CrearNodo(v, distancias[v]))
                minHeap.pos.append(v)
                salto.append(0)
                
            #Se define el nodo raiz 
            minHeap.pos[raiz] = raiz
            #La distancia del nodo raiz a el mismo es 0
            distancias[raiz] = 0
            minHeap.disminuirDistanciaMin(raiz, distancias[raiz])
            
            #El tamaño inicial del heap es el nro de vertices
            minHeap.tamaño = V;
            
            #El minHeap contiene todos los nodos de los vertices de los cuales
            #aun no se haya la minima distancia
            while minHeap.isEmpty() == False:
                #Extraemos el vertice raiz del Heap
                nuevoNodo = minHeap.extraerMinimo()
                vertice = nuevoNodo[0]
                
                #Se recorren todos los nodos adyacentes al vertice extraido (vertice)
                #y se actualiza sus valores de distancia
                for puntero in self.graph[vertice]:
                    v = puntero[0]
                    
                    #Si aun no se encuentra la distancia minima y la distancia hacia el vertice v a través de u
                    #es menor que las calculadas anteriormente
                    if minHeap.estaEnElHEap(v) and distancias[vertice] != sys.maxsize and puntero[1] + distancias[vertice] < distancias[v]:
                        #Se actualiza la distancia hacia el vertice "v"
                        distancias[v] = puntero[1] + distancias[vertice]
                        #Se sobreescribe el último salto para llegar al vertice v
                        salto[v] = vertice
                        #Se actualiza el valor de la distancia en el heap tambien
                        minHeap.disminuirDistanciaMin(v, distancias[v])
                        
            printRutas(distancias, V , salto )
                    
def main():
    
#Se corre el programa y se obtienen la distancia más corta desde
#el nodo raiz hasta el nodo indicado, la topologia/grafo debe ser 
#ingresado al programa a través de sus paquetes de estado de enlace

#  topologia a la
#  que pertenece   [[vertice,peso]]
#        ↓           ↓     ↓
#LSP(topologia1,0,[[1,5],[4,3]])
#               ↑
#       Router al que le pertenecen
#       el link-state packet 
#----------- Topologia Complejidad Baja Con Link-State Pakcets ----------------
    print("################ TOPOLOGÍA 1 ##################################\n")    
    topologia1 = Graph(8)
    #LSP[topologia a la que se quiere agregar conexión,el router principal,[Lista de conexiones]
    # Lista de conexiones = [router con conexión, peso]
    LSP(topologia1,0,[[1,5],[4,3]])
    LSP(topologia1,1,[[0,5],[2,6],[4,1]])
    LSP(topologia1,2,[[1,6],[3,3],[4,4]])
    LSP(topologia1,3,[[2,3],[4,2],[5,1]])
    LSP(topologia1,4,[[0,3],[1,1],[2,4],[3,2]])
    LSP(topologia1,5,[[3,1],[6,7]])
    LSP(topologia1,6,[[5,7],[7,8]])
    LSP(topologia1,7,[[6,8]])
    
    
    inicio = time.perf_counter_ns()
    topologia1.aplicaDijkstra(0)
    final = time.perf_counter_ns()
    tiempo1 = final - inicio
    print("\n")
    

    print("################ TOPOLOGÍA 2 #####################################\n")
    #--------- Topologia Complejidad Alta Con Link-State Pakcets----------------
    topologia2 = Graph(11)
    
    LSP(topologia2,0,[[1,3],[4,5]])
    LSP(topologia2,1,[[0,3],[4,2]])
    LSP(topologia2,2,[[6,2],[4,7],[3,3]])
    LSP(topologia2,3,[[2,3],[4,2],[5,2]])
    LSP(topologia2,4,[[0,5],[1,2],[2,7],[3,2]])
    LSP(topologia2,5,[[3,2],[6,9],[9,5],[10,10]])
    LSP(topologia2,6,[[2,2],[5,9],[7,11]])
    LSP(topologia2,7,[[6,11],[8,7],[9,4]])
    LSP(topologia2,8,[[7,7],[9,4]])
    LSP(topologia2,9,[[5,5],[7,4],[8,4]])
    LSP(topologia2,10,[[5,10]])
    
    
    inicio = time.perf_counter_ns()
    topologia2.aplicaDijkstra(0)
    final = time.perf_counter_ns()
    tiempo2 = final - inicio
    pid = os.getpid()
    py = psutil.Process(pid)
    memory_used = py.memory_info()
    MB = (memory_used[0]/(10**6))
    #Son grafos no dirigidos, así que las conexiones se repiten
    print(f"""

*************************** MÉTRICAS *******************************

El tiempo de ejecución de la topología 1 en microsegundos es: {tiempo1/1000}
El tiempo de ejecución de la topología 2 en microsegundos es: {tiempo2/1000}

                                      numero de aristas
                                              ↓                                       
La complejidad del algoritmo Dijkstra es: O((|A|+|V|) log |V|) 
                                                  ↑
                                           numero de vertices
                            
                                    Vertices\tAristas\t\tNro Bucles
Tamaño de los Grafos: Topología 1\t   {topologia1.V}\t\t  {topologia1.aristas//2}\t\t\t6
                      Topología 2\t   {topologia2.V}\t\t  {topologia2.aristas//2}\t\t\t7

El algoritmo usa {MB:.2f} MB de memoria para poder resolver ambas topologías 
implementadas en el documento actual.
""")
    
main()
  

#MÉTRICAS
#COMPLEJIDAD
#TIEMPO
#TAMAÑO GRAFO
#NUMERO DE BUCLES
#PORCENTAJE DE USO DE MEMORIA 
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
    
    
    
    
