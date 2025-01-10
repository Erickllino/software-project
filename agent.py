import heapq
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point

class ExampleAgent(BaseAgent):

    def __init__(self, id=0, yellow=False):
        super().__init__(id, yellow)
        self.targets_atuais = []
        self.path = None
        self.temp_target = None
        self.i = 0

    def decision(self):
        # Verifica se existem alvos
        if len(self.targets) == 0:
            return

        
        # Obtém posições dos oponentes
        opponent_position = [(op.x, op.y) for op in self.opponents.values()]

        # Verifica se há obstáculos perto da posição atual
        obs = self.is_near_opponent(self.pos, opponent_position, max_distance=0.25)

        if self.targets_atuais != self.targets:  # Se não houver obstáculos
            self.targets_atuais = self.targets
            '''# Move diretamente ao alvo
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.targets[0])
            else:'''
            # Usa A* para encontrar um caminho
            start = (self.robot.x, self.robot.y)
            goal = (self.targets[0].x, self.targets[0].y)
            
            self.path = self.a_star(start, goal, opponent_position)
            self.i = 0
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



                
                print(f'-----------------------------\nindo para {point}')

                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.temp_target)
                
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)

                print(f'\nheuristica: {self.heuristic(self.temp_target ,self.pos)}\n')

                if self.heuristic(self.temp_target ,self.pos) >= 0.05:
                    print(f'posição atual: {self.pos}, ponto atual: {point}\n--------------------')
                    self.temp_target = point
                    return
                if self.heuristic(self.temp_target ,self.pos) < 0.05:
                    print(f'i : {self.i}')
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

    def a_star(self, start, goal, opponent_position):
        """
        Implementação do A* para encontrar o caminho até o objetivo.
        """
        # Movimentos possíveis: cima, baixo, esquerda, direita
        neighbors = [(0, -0.09), (0, 0.09), (-0.09, 0), (0.09, 0)]

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
                
                return path[::-1]  # Retorna o caminho na ordem correta
            
            # Explora os vizinhos
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Verifica se o vizinho está livre de obstáculos
                if not self.is_near_opponent(neighbor, opponent_position):
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        
                        g_score[neighbor] = tentative_g_score
                        priority = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return None  # Retorna None se nenhum caminho for encontrado

    def is_near_opponent(self, point, opponent_position, max_distance=0.18):
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
