from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad
        self.siendo_atendida = False


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class Estacion(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class RobotLimpieza(Agent):


    def __init__(self, unique_id, model, posiciones_estaciones, posiciones_celdas_sucias):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.pos_tarea = None
        self.pos_estaciones = posiciones_estaciones
        self.pos_celdas_sucias = posiciones_celdas_sucias
        self.celdas_limpiadas = []
        self.prendido = True
        self.estacion_cargas = 0

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.carga -= 1
        self.sig_pos = celda_a_limpiar.pos
        lista_temp = [pos_suciedad for pos_suciedad in self.pos_celdas_sucias if pos_suciedad != celda_a_limpiar.pos]
        self.pos_celdas_sucias = lista_temp
        self.celdas_limpiadas.append(celda_a_limpiar.pos)
        if celda_a_limpiar.pos == self.pos_tarea:
            self.pos_tarea = None

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        self.sig_pos = self.random.choice(lista_de_vecinos).pos

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        # return [vecino for vecino in lista_de_vecinos
        #                 if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias

    def step(self):

        vecinos_todos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=True)

        vecinos = [vecino for vecino in vecinos_todos if not isinstance(vecino, (Mueble, RobotLimpieza, Estacion))]

        vecinos_con_estaciones_y_robots = [vecino for vecino in vecinos_todos if not isinstance(vecino, (Mueble))]

        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        vecinos_robots_comunicacion = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False, radius=4)

        for celda in vecinos_robots_comunicacion:

            if isinstance(celda, RobotLimpieza):
                lista_temp_limpiadas = [limpiado for limpiado in celda.celdas_limpiadas if limpiado not in self.celdas_limpiadas]
                self.celdas_limpiadas.extend(lista_temp_limpiadas)

        lista_temp_sucias = [suciedad for suciedad in self.pos_celdas_sucias if suciedad not in self.celdas_limpiadas]
        self.pos_celdas_sucias = lista_temp_sucias

        if self.prendido == True:

            if (self.pos_tarea == None):

                x_actual, y_actual = self.pos

                hipotenusa_suciedad_cercana = 1000
                suciedad_cercana = None

                pos_sucias_adyacentes = [suciedad_adyacente.pos for suciedad_adyacente in celdas_sucias]

                for suciedad in self.pos_celdas_sucias:

                    if suciedad not in pos_sucias_adyacentes:

                        x_suciedad, y_suciedad = suciedad

                        x_temp = x_suciedad - x_actual
                        y_temp = y_suciedad - y_actual

                        hipotenusa_temp = np.sqrt((x_temp * x_temp) + (y_temp * y_temp))

                        if hipotenusa_temp < hipotenusa_suciedad_cercana:

                            hipotenusa_suciedad_cercana = hipotenusa_temp
                            suciedad_cercana = suciedad
                    
                self.pos_tarea = suciedad_cercana

            if self.pos in self.pos_estaciones:

                if self.carga < 100:

                    self.carga +=25

                    if self.carga > 100:
                        self.carga = 100
                        self.sig_pos = self.pos

                else:

                    self.pos_tarea = None
                    self.estacion_cargas += 1
                    self.seleccionar_nueva_pos(vecinos)

            elif self.carga <= 15 and (self.pos_tarea not in self.pos_estaciones):
                
                x, y = self.pos

                hipotenusa_estacion_cercana = 1000
                estacion_cercana = None

                for estacion in self.pos_estaciones:

                    x_estacion, y_estacion = estacion

                    x_temp = x_estacion - x
                    y_temp = y_estacion - y

                    hipotenusa_temp = np.sqrt((x_temp * x_temp) + (y_temp * y_temp))

                    if hipotenusa_temp < hipotenusa_estacion_cercana:

                        hipotenusa_estacion_cercana = hipotenusa_temp
                        estacion_cercana = estacion
                
                self.pos_tarea = estacion_cercana
                self.seleccionar_nueva_pos(vecinos)


            elif len(celdas_sucias) != 0:

                self.limpiar_una_celda(celdas_sucias)

            elif self.pos_tarea != None:

                if self.pos_tarea in self.pos_estaciones:

                    estacion_encontrada = False

                    for vecino in vecinos_con_estaciones_y_robots:

                        if vecino.pos == self.pos_tarea:

                            if isinstance(vecino, RobotLimpieza):
                                estacion_encontrada = True
                                self.sig_pos = self.pos
                            else:
                                estacion_encontrada = True
                                self.sig_pos = self.pos_tarea

                    if estacion_encontrada == False:

                        x_tarea, y_tarea = self.pos_tarea

                        hipotenusa_optima = 1000
                        pos_optima = self.pos


                        for vecino in vecinos:

                            x_vecino, y_vecino = vecino.pos

                            x_temp = x_tarea - x_vecino
                            y_temp = y_tarea - y_vecino

                            hipotenusa_temp = np.sqrt((x_temp * x_temp) + (y_temp * y_temp))

                            if hipotenusa_temp < hipotenusa_optima:

                                hipotenusa_optima = hipotenusa_temp
                                pos_optima = vecino.pos

                        
                        if pos_optima == self.pos:
                            self.seleccionar_nueva_pos(vecinos)
                        else:   
                            self.sig_pos = pos_optima

                        
                elif self.pos_tarea in self.pos_celdas_sucias:    

                    x_tarea, y_tarea = self.pos_tarea

                    hipotenusa_optima = 1000
                    pos_optima = self.pos


                    for vecino in vecinos:

                        x_vecino, y_vecino = vecino.pos

                        x_temp = x_tarea - x_vecino
                        y_temp = y_tarea - y_vecino

                        hipotenusa_temp = np.sqrt((x_temp * x_temp) + (y_temp * y_temp))

                        if hipotenusa_temp < hipotenusa_optima:

                            hipotenusa_optima = hipotenusa_temp
                            pos_optima = vecino.pos

                    
                    if pos_optima == self.pos:
                        self.seleccionar_nueva_pos(vecinos)
                    else:   
                        self.sig_pos = pos_optima

                else:

                    self.pos_tarea = None

            else:

                self.seleccionar_nueva_pos(vecinos)

        if self.pos_celdas_sucias == []:

            self.sig_pos = self.pos
            self.prendido = False


    def advance(self):


        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0 and not (self.pos in self.pos_estaciones and self.pos_tarea in self.pos_estaciones) and self.prendido == True:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)


class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        # Posicionamiento de estaciones
        num_estaciones = 4
        posiciones_estaciones = [(5,5), (5, 15), (15, 5), (15, 15)]

        for id, pos in enumerate(posiciones_estaciones):
            estacion = Estacion(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)


        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self, posiciones_estaciones, posiciones_celdas_sucias)
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias, "Movimientos": get_movimientos, "Cargas totales": get_estacion_cargas}
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(model: Model) -> int:

    sum_movs = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                sum_movs += obj.movimientos
    return sum_movs
    
def get_estacion_cargas(model: Model) -> int:

    sum_cargas_estacion = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                sum_cargas_estacion += obj.estacion_cargas
    return sum_cargas_estacion
    

    # else:
    #    return 0
