class Joint():
    def __init__(self,name):
        self._name = name
        self._position = 0.0
        self._velocity = 0.0
        self._effort = 0.0
    
    @property
    def name(self) -> str:
        return self._name
    
    @property
    def position(self) -> float:
        return self._position
    
    @property
    def velocity(self) -> float:
        return self._velocity

    @property
    def effort(self) -> float:
        return self._effort
    
    @position.setter
    def position(self, value: float):
        self._position = value

    @velocity.setter
    def velocity(self, value: float):
        self._velocity = value

    @effort.setter
    def effort(self, value: float):
        self._effort = value

    # --- Utility method per aggiornare tutto insieme ---
    def update_state(self, position: float, velocity: float, effort: float):
        self._position = position
        self._velocity = velocity
        self._effort = effort