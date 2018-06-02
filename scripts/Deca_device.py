import DW1000Constants as C

class DecaDevice:
    TAG     = 0
    ANCHOR  = 1
    def __init__(self, address, type_of_tag):
        self.address                = address
        self.type                   = type_of_tag
        self.is_inactive            = False
        self.sequenceNumber         = 0
        self.expectedMessage        = C.POLL_ACK if (self.type == DW1000Device.ANCHOR) else C.POLL
        
    def deletePreviousSequenceData(self):
        for i in self.timestamps:
            for j in i.keys():
                if j != self.sequenceNumber-1:
                    del i[j]

    def is_inactive(self):
        return is_inactive

    def activate(self):
        self.is_inactive = False

    def deactivate(self):
        self.is_inactive = True

    def __str__(self):
        str = ""
        d   = self.__dict__
        for key in d:
            str += "{}: {}\n".format(key, d[key])