import heapq
import numpy as np
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import time
import math

class ExampleAgent(BaseAgent):

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.targets_atuais = []
        self.teammate_objective = dict()
        self.target_assigments = dict()
        self.paths = dict()
        self.temp_target = None
        self.last_point_check = dict()
        self.last_path_update = time.time()
        self.min_dist = 0.25

    def decision(self):
        if len(self.targets) == 0:                                              # Verifica se existem alvos
            return
        current_time = time.time()                                              # Obtém o tempo atual
        opponent_position = [(op.x, op.y) for op in self.opponents.values()]    # Obtém posições dos oponentes



        if self.targets_atuais != self.targets:
            self.targets_atuais = self.targets
            self.teammate_objective = {teammate: {'targets':[],  'temp_target_id':0 , 'temp_target': list(), 'path': list()} for teammate in self.teammates}
            self.assign_targets()

            for assigned_id, objective in self.teammate_objective.items():
                if objective['targets'] != []:
                    target = objective['targets'][0]
                    goal = (target.x, target.y)
                    agent = (self.teammates[assigned_id].x, self.teammates[assigned_id].y)
                    self.teammate_objective[assigned_id]['path'] = self.a_star(agent, goal, opponent_position)
                else:
                    self.assign_targets()
                    
                    pass
            
        else:
            if self.targets_atuais is not None:
                for assigned_id, objective in self.teammate_objective.items():
                    if objective['targets'] != []:
                        target = objective['targets'][0]
                        goal = (target.x, target.y)
                        agent = (self.teammates[assigned_id].x, self.teammates[assigned_id].y)
                        
                        
                        if self.teammate_objective[assigned_id]['path'] is not None:
                            self.go_to_next_point(assigned_id)

                        if self.check_col(assigned_id):

                            # Reorganiza o caminho caso encontre uma colisão
                            self.reorganize_path(current_time, assigned_id, goal, opponent_position)
                    else:
                        self.target_assigments = self.assign_targets()
                        self.go_to_closest_target(assigned_id)
            else:
                # Caso não exista caminho, para o robô
                print('Está indo reto')
                goal = Point(goal[0], goal[1])
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.teammates[assigned_id], goal)
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
                return

        
        
        
        




    def check_col(self, assigned_id):

        prediction_time = 0.2  # segundos
        safe_distance = 0.6  # Distância mínima para evitar colisão
        
        agent = self.teammates[assigned_id]
        position = Point(agent.x, agent.y)
        velocity = Point(agent.v_x, agent.v_y)
        
        # Trajetória do agente (posição futura)
        future_pos = Point(position[0] + velocity[0] * prediction_time, position[1] + velocity[1] * prediction_time)

        # Detecta os oponentes próximos e calcula suas posições futuras
        for opponent_id, opponent in self.opponents.items():
            # Previsão da posição futura do oponente
            opponent_future_pos = Point(opponent.x + opponent.v_x * prediction_time, opponent.y + opponent.v_y * prediction_time)

            # Calcula a distância entre o agente e o oponente no futuro
            distance_to_opponent = future_pos.dist_to(opponent_future_pos)

            # Se a distância for menor que a distância segura, há risco de colisão
            if distance_to_opponent < safe_distance:
        
                return True
            
            return False
                
                
                
        """
                # Ajuste na velocidade para desviar
                self.set_vel(avoidance_vector * self.vel.length())

                # Ajusta a direção do agente (angulação)
                angle_to_avoidance = math.atan2(avoidance_vector.y, avoidance_vector.x)
                self.set_angle_vel(angle_to_avoidance - self.body_angle)  # Alinha o agente com a direção do desvio
                return  # O agente já está desviando, então terminamos aqui."""

    def post_decision(self):
        pass
    
    def reorganize_path(self, current_time, assigned_id, goal, ops):
        agent = (self.teammates[assigned_id].x, self.teammates[assigned_id].y)
        
        # Usa A* para encontrar um caminho até o objetivo
        self.teammate_objective[assigned_id]['path'] = self.a_star(agent, goal, ops)
        self.teammate_objective[assigned_id]['temp_target_id'] = 0
        self.last_path_update = current_time

    def assign_targets(self):
        target_assigments = {target: None for target in self.targets}
        for target in self.targets:
            closest_teammate_id = None
            closest_distance = float('inf')  # Seta a distância máxima no infinito
            for teammate_id, teammate in self.teammates.items():
                distance = np.linalg.norm([target.x - teammate.x, target.y - teammate.y])
                if distance < closest_distance:
                    closest_distance = distance
                    closest_teammate_id = teammate_id
            self.teammate_objective[closest_teammate_id]['targets'].append(target)
            target_assigments[closest_teammate_id] = target
        return target_assigments

    def go_to_closest_target(self, assigned_id):
        closest_distance = float('inf')
        closest_target = ()
        agent = self.teammates[assigned_id]
        start = (agent.x, agent.y)
        for target in self.targets:
            if self.heuristic(start, target) < closest_distance:
                closest_target = target.x, target.y
                closest_distance = self.heuristic(start, target)
        self.teammate_objective[assigned_id]['targets'].append(target)

    def go_to_next_point(self, assigned_id):
        path = self.teammate_objective[assigned_id]['path']
        temp_target = self.teammate_objective[assigned_id]['temp_target']
        temp_target_id = self.teammate_objective[assigned_id]['temp_target_id']

        if temp_target_id == len(path):
            path = None
            temp_target_id = 0
            temp_target = None
            return
        else:
            point = path[temp_target_id]

        temp_target = Point(point[0], point[1])

        if self.heuristic(temp_target, self.pos) < 0.05:
            self.teammate_objective[assigned_id]['temp_target_id'] += 1

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.teammates[assigned_id], temp_target)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    def heuristic(self, a, b):
        """Função heurística (distância euclidiana)"""
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

    def resumir_caminho(self, caminho, tolerancia=0.05):
        return self.douglas_peucker(caminho, tolerancia)

    def douglas_peucker(self, caminho, tolerancia):
        if len(caminho) < 3:
            return caminho 

        def distancia_perpendicular(ponto, linha_inicio, linha_fim):
            if linha_inicio == linha_fim:
                return ((ponto[0] - linha_inicio[0])**2 + (ponto[1] - linha_inicio[1])**2)**0.5
            else:
                num = abs((linha_fim[1] - linha_inicio[1]) * ponto[0] - 
                        (linha_fim[0] - linha_inicio[0]) * ponto[1] + 
                        linha_fim[0] * linha_inicio[1] - linha_fim[1] * linha_inicio[0])
                den = ((linha_fim[1] - linha_inicio[1])**2 + (linha_fim[0] - linha_inicio[0])**2)**0.5
                return num / den

        linha_inicio = caminho[0]
        linha_fim = caminho[-1]
        max_dist = 0
        index = 0

        for i in range(1, len(caminho) - 1):
            dist = distancia_perpendicular(caminho[i], linha_inicio, linha_fim)
            if dist > max_dist:
                max_dist = dist
                index = i

        if max_dist > tolerancia:
            parte1 = self.douglas_peucker(caminho[:index + 1], tolerancia)
            parte2 = self.douglas_peucker(caminho[index:], tolerancia)
            return parte1[:-1] + parte2
        else:
            return [linha_inicio, linha_fim]

    def a_star(self, start, goal, opponent_position):
        """
        Implementação do A* para encontrar o caminho até o objetivo.
        """
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

                dist = 0.25
                if self.is_near_goal(current, goal):
                    dist = 0.1

                # Verifica se o vizinho está livre de obstáculos
                if not self.is_near_opponent(neighbor, opponent_position, dist) or self.is_goal_reached(neighbor, goal):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        g_score[neighbor] = tentative_g_score
                        priority = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return None  # Retorna None se nenhum caminho for encontrado

    def is_near_opponent(self, point, opponent_position, max_distance):
        """
        Verifica se a posição 'point' está próxima a algum obstáculo.
        """
        for opponent in opponent_position:
            distance = self.heuristic(point, opponent)
            if distance < max_distance:
                return True  # Obstáculo próximo
        return False  # Nenhum obstáculo próximo

    def is_goal_reached(self, current, goal, tolerance=0.12):
        """
        Verifica se o ponto atual está próximo o suficiente do objetivo.
        """
        return abs(current[0] - goal[0]) < tolerance and abs(current[1] - goal[1]) < tolerance

    def is_near_goal(self, current, goal, max_distance=0.4):
        """
        Verifica se o ponto calculado no A* está próximo ao objetivo, para que possamos reduzir a velocidade ou alterar o comportamento.

        Parâmetros:
        - current: tupla (x, y) representando a posição atual.
        - goal: tupla (x, y) representando o objetivo.
        - max_distance: distância máxima para considerar o ponto próximo ao objetivo.

        Retorna:
        - True se a distância entre current e goal for menor que max_distance.
        - False caso contrário.
        """
        # Calcula a distância entre o ponto atual e o objetivo
        distance = self.heuristic(current, goal)
        
        # Verifica se a distância é menor que a distância máxima
        return distance < max_distance