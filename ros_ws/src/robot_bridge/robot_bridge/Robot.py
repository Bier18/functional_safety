from joint import Joint

class Robot:
    def __init__(self, name: str, joint_names: list[str]):
        self._name = name
        # Creo i giunti come oggetti Joint in un dizionario
        self._joints = {j_name: Joint(j_name) for j_name in joint_names}

    # --- Getters ---
    @property
    def name(self) -> str:
        return self._name

    @property
    def joints(self) -> dict[str, Joint]:
        return self._joints

    # Restituisce un giunto specifico per nome
    def get_joint(self, name: str) -> Joint | None:
        return self._joints.get(name, None)

    # Aggiorna lo stato di un giunto specifico
    def update_joint(self, name: str, position: float, velocity: float, effort: float):
        joint = self.get_joint(name)
        if joint:
            joint.update_state(position, velocity, effort)
        else:
            raise ValueError(f"Joint {name} not found in robot {self._name}")

    # Aggiorna tutti i giunti insieme (lista di tuple: [(name, pos, vel, eff), ...])
    def update_all_joints(self, states: list[tuple[str, float, float, float]]):
        for name, pos, vel, eff in states:
            self.update_joint(name, pos, vel, eff)