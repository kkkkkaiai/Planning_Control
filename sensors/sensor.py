class Sensor:
    def __init__(self, ix, iu, time=0.0) -> None:
        self._time = time
        self._ix = ix
        self._iu = iu

    def forward(self, x, u, idx=None, discrete=True):
        raise NotImplementedError
    
    def diff(self):
        raise NotImplementedError
    