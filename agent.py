from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from collections import deque
import heapq

import random

class ExampleAgent(BaseAgent):

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)

    def decision(self):
        global n

        if len(self.targets) == 0:
            return


        # self.teammates é a lista de agentes
        # self.opponents é a lista dos oponentes

        
        obs = self.detect_obstacles()
        if not obs:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            
        else:
            nada  =Point(100, 100)
            start = (self.robot.x, self.robot.y)
            goal = (self.targets[0].x, self.targets[0].y)
            path = self.a_star(start, goal, self.opponents)
            if path:
                next_point = path[1]  # Próximo ponto no caminho
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, Point(next_point[0], next_point[1]))
            else:
                target_velocity, target_angle_velocity = 0, 0  # Parar se não houver caminho
            #arget_velocity, target_angle_velocity = Navigation.goToPoint(self.robot,nada)  # Parar se não houver caminho
            
        
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

        return

    def post_decision(self):
        pass


    

    def heuristic(self,a, b):
        """
        Função heurística para estimar a distância de a até b (usando a distância de Manhattan).
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self,start, goal, obstacles):
        """
        Implementação do algoritmo A* para encontrar o caminho do agente até o alvo.

        Parâmetros:
        - grid: matriz 2D representando o mapa (0 = espaço livre, 1 = obstáculo).
        - start: tupla (x, y) com a posição inicial do agente.
        - goal: tupla (x, y) com a posição do alvo.

        Retorna:
        - Lista de tuplas com o caminho do agente até o alvo ou None se não houver caminho.
        """
        # Movimentos possíveis: cima, baixo, esquerda, direita
        neighbors = [(0, -0.1), (0, 0.1), (-0.1, 0), (0.1, 0)]

        obstacle_positions = [(op.x, op.y) for op in obstacles.values()]
        print(f'posicao dos obstaculos: {obstacle_positions}')
        # Fila de prioridade (custo, posição atual)
        open_set = []
        heapq.heappush(open_set, (0, start))

        # Dicionário para rastrear os menores custos até um nó
        g_score = {start: 0}

        # Dicionário para rastrear o caminho
        came_from = {}

        while open_set:
            # Nó com menor custo estimado
            _, current = heapq.heappop(open_set)

            # Verifica se atingimos o objetivo
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Retorna o caminho na ordem correta

            # Explora os vizinhos
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                # Verifica se o vizinho não está nos obstáculos
                if neighbor not in obstacle_positions:
                    # Calcula o custo para alcançar o vizinho
                    tentative_g_score = g_score[current] + 1

                    # Se o caminho atual for melhor, atualiza
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        g_score[neighbor] = tentative_g_score
                        priority = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return None  # Retorna None se não houver caminho

    def detect_obstacles(self):
        # Método para determinar se há obstáculos no caminho
        # Exemplo: verificar proximidade de oponentes
        return any(self.is_near_opponent(op) for op in self.opponents)
    
    def is_near_opponent(self, opponent):
        # Método para verificar se um oponente está próximo
        print(f'posicao do robo {self.pos}')
        
        
        distance = ((self.pos[0] - self.opponents[opponent].x)**2 + (self.pos[1] - self.opponents[opponent].y)**2)**0.5
        print(f'distancia relativa ao obstaculo {opponent} : {distance}')
        return distance < 0.25  # Ajuste o raio conforme necessário

    def get_obstacle_positions(self):
        # Retorna posições dos obstáculos (oponentes)
        return [op.pos for op in self.opponents]
