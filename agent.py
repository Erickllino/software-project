import heapq
import numpy as np
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
import time

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
        current_time = time.time()                                              # Obtém o tempo atua
        opponent_position = [(op.x, op.y) for op in self.opponents.values()]    # Obtém posições dos oponentes


        


        if self.targets_atuais != self.targets:
            self.targets_atuais = self.targets
            """
            Aqui Estou atualizando o caminho ddos agentes, toda vez que o set dos alvos
            muda. Teoricamente, deve acontecer somente uma vez por isso o self.teammate_objective foi setado aqui
            Se eu tiver uma ideia melhor ou mais organizada --> Fazer ASAP
            
            
            """
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
                    print(f'o agente {assigned_id} esta parado')
                    # Se o agente não tiver nenhum alvo

                    pass

            
        else:
            
            
            if self.targets_atuais is not None:
                

                for assigned_id, objective in self.teammate_objective.items():
                    if objective['targets'] != []:
                        target = objective['targets'][0]
                        goal = (target.x, target.y)
                        agent = (self.teammates[assigned_id].x, self.teammates[assigned_id].y)
                        
                        # Reorganizar o Caminho -- obstaculos dinamicos
                        self.reorganize_path(current_time, assigned_id, goal, opponent_position)

                                
                        
                        if self.teammate_objective[assigned_id]['path'] is not None:
                            
                            self.go_to_next_point(assigned_id)
                    
                    else:
                        self.target_assigments = self.assign_targets()
                        self.go_to_closest_target(assigned_id)
                        # Se o agente nao tiver nenhum alvo

                        pass
                    
                    
                            
            else:
                # Caso não exista caminho, para o robô
                print('Ta so ido reto')
                goal = Point(goal[0], goal[1])
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.teammates[assigned_id], goal)
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
                return

              

    def post_decision(self):
        pass
    

    def reorganize_path(self, current_time, assigned_id, goal,ops):

        agent = (self.teammates[assigned_id].x, self.teammates[assigned_id].y)
        if current_time - self.last_path_update>=3:  # Se os alvos nao forem os mesmos ou passou 3 segundos
            # Usa A* para encontrar um caminho até o objetivo

            
            self.teammate_objective[assigned_id]['path'] = self.a_star(agent, goal, ops)
            self.teammate_objective[assigned_id]['temp_target_id'] = 0
            self.last_path_update = current_time            
                        
                        

    def assign_targets(self):
        target_assigments = {target: None for target in self.targets}
        

        for target in self.targets:
            closest_teammate_id = None
            closest_distance = float('inf') #Seta a distancia maxima no infinito

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
        agent =self.teammates[assigned_id]
        start = (agent.x, agent.y)
        for target in self.targets:
            if self.heuristic(start, target) < closest_distance:
                closest_target = target.x, target.y
                closest_distance = self.heuristic(start, target)

        
        self.teammate_objective[assigned_id]['targets'].append(target)


        
        

    def go_to_next_point(self, assigned_id):


        # Dict --> {assigned_id : {targets:list, temp_target_id:0, temp_target: Tuple, path: List}}

        path = self.teammate_objective[assigned_id]['path']
        temp_target = self.teammate_objective[assigned_id]['temp_target']
        temp_target_id = self.teammate_objective[assigned_id]['temp_target_id']

        if temp_target_id == len(path):
                path= None
                temp_target_id = 0
                temp_target = None

                return
        else:
              
            point = path[temp_target_id]

        temp_target = Point(point[0], point[1])

         
        if self.heuristic(temp_target ,self.pos) < 0.05:
                      
            self.teammate_objective[assigned_id]['temp_target_id'] += 1

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.teammates[assigned_id], temp_target)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)
             


    def heuristic(self, a, b):
        """Função heurística (distância euclidiana)"""
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5
    
    def resumir_caminho(self,caminho, tolerancia=0.05):
        return self.douglas_peucker(caminho, tolerancia)

    def douglas_peucker(self,caminho, tolerancia):
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

        Parâmetros:
        - point: tupla (x, y) com a posição do ponto a ser verificado.
        - opponent_position: lista de posições (x, y) dos obstáculos.
        - max_distance: distância máxima para considerar um obstáculo.

        Retorna:
        - True se a posição 'point' estiver a uma distância menor que max_distance de algum obstáculo.
        - False caso contrário.
        """
        for opponent in opponent_position:
            distance = self.heuristic(point, opponent)
            if distance < max_distance:
                return True  # Obstáculo próximo
        return False  # Nenhum obstáculo próximo
    
    def is_near_goal(self, current, goal, max_distance = 0.4):
        """
        Verifica se o ponto calculado no A* esta proximo ao objetivo, paara que possamos reduzir a
        
        Parametros:
        - Current:
        - goal
        - max_distance

        Retorna:
        - True
        - False


        ---- Termminar e escrever -----
        
        """

        distance = self.heuristic(current,goal)
        if distance < max_distance:
            return True
        return False
        
        

    def is_goal_reached(self, current, goal, tolerance=0.12):
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
