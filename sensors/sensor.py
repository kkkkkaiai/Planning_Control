class Sensor:
    def __init__(self, time=0.0) -> None:
        self._time = time

    def forward(self, x, u, idx=None, discrete=True):
        raise NotImplementedError
    
    def diff(self):
        raise NotImplementedError
    