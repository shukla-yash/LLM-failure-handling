class DetectionFailure(Exception):
    def __init__(self, name, reason=''):
        self.name = name
        self.reason = reason


class PickFailure(Exception):
    def __init__(self, name, reason=''):
      self.name = name
      self.reason = reason


class PlaceFailure(Exception):
    def __init__(self, name, reason=''):
        self.name = name
        self.reason = reason


class PushFailure(Exception):
    def __init__(self, name, reason):
        self.name = name
        self.reason = reason