import heapq
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import time

class ExampleAgent(BaseAgent):

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.targets_atuais = []
        self.path = None
        self.temp_target = None
        self.i = 0
        self.last_path_update = time.time()
        self.min_dist = 0.25

    def decision(self):
        # Verifica se existem alvos
        if len(self.targets) == 0:
            return

        
        # Obtém posições dos oponentes
        opponent_position = [(op.x, op.y) for op in self.opponents.values()]

        obs = self.is_near_opponent(self.pos, opponent_position, max_distance=self.min_dist)
        
        current_time = time.time()

        

        if self.targets_atuais != self.targets or current_time - self.last_path_update>=2:  # Se os alvos nao forem os mesmos ou passou 3 segundos
            self.targets_atuais = self.targets
            
            # Usa A* para encontrar um caminho
            start = (self.robot.x, self.robot.y)
            goal = (self.targets[0].x, self.targets[0].y)
            
            self.path = self.a_star(start, goal, opponent_position)
            self.i = 0
            self.last_path_update = current_time
            self.min_dist = 0.25
            


            
        else:
            if self.path is not None:
                
                if self.i == len(self.path) - 1:
                        self.i = 0
                        self.path = None
                        self.temp_target = None
                        return
                else:   
                    point = self.path[self.i]
                    
                self.temp_target = Point(point[0], point[1])



                
                

                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.temp_target)
                
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)

                

                if self.heuristic(self.temp_target ,self.pos) >= 0.05:
                    
                    self.temp_target = point
                    return
                if self.heuristic(self.temp_target ,self.pos) < 0.05:
                    
                    self.i += 1
                    return
                
                
                    
            else:
                # Caso não exista caminho, para o robô
                print('Ta so ido reto')
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
                return

            '''# Define a velocidade final
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)'''

    def post_decision(self):
        pass

    def heuristic(self, a, b):
        """Função heurística (distância euclidiana)"""
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5
    
    def resumir_caminho(self,caminho):
        if len(caminho) < 3:
            return caminho  # Se tiver menos de 3 pontos, não dá para simplificar.

        # Inicializar lista de pontos resumidos
        caminho_resumido = [caminho[0]]  # Começa com o primeiro ponto

        # Função para verificar se 3 pontos são colineares
        def sao_colineares(p1, p2, p3):
            # Determinante para verificar colinearidade
            # | x1 y1 1 |
            # | x2 y2 1 | = 0 indica colinearidade
            # | x3 y3 1 |
            return (p2[0] - p1[0]) * (p3[1] - p1[1]) == (p3[0] - p1[0]) * (p2[1] - p1[1])

        # Verificar colinearidade ao longo do caminho
        for i in range(1, len(caminho) - 1):
            if not sao_colineares(caminho[i - 1], caminho[i], caminho[i + 1]):
                caminho_resumido.append(caminho[i])

        # Adicionar o último ponto
        caminho_resumido.append(caminho[-1])

        return caminho_resumido
    
    def a_star(self, start, goal, opponent_position):
        """
        Implementação do A* para encontrar o caminho até o objetivo.
        """
        # Movimentos possíveis: cima, baixo, esquerda, direita
        n = 0.09
        neighbors = [(0, -n), (0, n), (-n, 0), (n, 0), (-n, -n), (-n, n), (n, -n), (n, n)]

        # Inicializa a lista de prioridades e os scores
        open_set = []
        heapq.heappush(open_set, (0, start))
        g_score = {start: 0}
        came_from = {}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # Verifica se o objetivo foi alcançado
            if self.is_goal_reached(current, goal):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                
                return self.resumir_caminho(path[::-1])  # Retorna o caminho na ordem correta
            
            # Explora os vizinhos
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Verifica se o vizinho está livre de obstáculos
                if not self.is_near_opponent(neighbor, opponent_position) or self.is_goal_reached(neighbor, goal):
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        
                        g_score[neighbor] = tentative_g_score
                        priority = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return None  # Retorna None se nenhum caminho for encontrado

    def is_near_opponent(self, point, opponent_position, max_distance=0.25):
        """
        Verifica se a posição 'point' está próxima a algum obstáculo.

        Parâmetros:
        - point: tupla (x, y) com a posição do ponto a ser verificado.
        - opponent_position: lista de posições (x, y) dos obstáculos.
        - max_distance: distância máxima para considerar um obstáculo.

        Retorna:
        - True se a posição 'point' estiver a uma distância menor que max_distance de algum obstáculo.
        - False caso contrário.
        """
        for opponent in opponent_position:
            distance = ((point[0] - opponent[0])**2 + (point[1] - opponent[1])**2)**0.5
            if distance < max_distance:
                return True  # Obstáculo próximo
        return False  # Nenhum obstáculo próximo

    def is_goal_reached(self, current, goal, tolerance=0.1):
        """
        Verifica se o ponto atual está próximo o suficiente do objetivo.

        Parâmetros:
        - current: ponto atual como tupla (x, y).
        - goal: ponto objetivo como tupla (x, y).
        - tolerance: tolerância para considerar que o objetivo foi alcançado.

        Retorna:
        - True se o ponto atual estiver dentro da tolerância do objetivo.
        - False caso contrário.
        """
        return abs(current[0] - goal[0]) < tolerance and abs(current[1] - goal[1]) < tolerance
