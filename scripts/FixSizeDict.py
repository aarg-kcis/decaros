from collections import OrderedDict

class FixSizeDict(OrderedDict):
    def __init__(self, max=0):
            self._max = max
            super(FixSizeDict, self).__init__()

    def __setitem__(self, key, value):
        OrderedDict.__setitem__(self, key, value)
        if self._max > 0:
            if len(self) > self._max:
                self.popitem(False)