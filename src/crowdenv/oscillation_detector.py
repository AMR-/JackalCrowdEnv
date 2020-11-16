class OscillationDetect:
    def __init__(self):
        self.velocities = []
        self.threshold = 5
        self.max_velocities = 5
        self.last_status = None

    def reset(self):
        self.velocities.clear()

<<<<<<< HEAD
=======
    # noinspection PyMethodMayBeStatic
>>>>>>> 25ad69f87fc0158bde48b4eddcbe599de49c5edb
    def convert(self, vel: int):
        if vel == 0 or vel == 1:
            return -1
        elif vel == 4 or vel == 5:
            return 1
        else:
            return 0

    def recognise(self, velocity: int):
        if len(self.velocities) < self.max_velocities:
            self.velocities.append(velocity)
        else:
            self.velocities.pop(0)
            self.velocities.append(velocity)

        if len(self.velocities) < self.threshold:
            return False
        else:
            situation = []
            for vel in self.velocities:
                situation.append(self.convert(vel))

            a = situation[0] == situation[1]
            b = situation[3] == situation[4]
            c = situation[0] == -situation[3]
            d = situation[0] != 0 and situation[3] != 0
            e = self.last_status is not True
            if a and b and c and d and e:
                self.last_status = True
                return True
            else:
                self.last_status = False
                return False
