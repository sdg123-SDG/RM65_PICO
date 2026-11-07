"""Edge-triggered follow latch; no robot or network access."""

class FollowLatch:
    def __init__(self, threshold=0.55, release=0.1):
        self.threshold = threshold
        self.release = release
        self.active = False
        self._start_ready = False
        self._stop_ready = False

    def update(self, start, stop, valid=True):
        if not valid:
            self.active = False
            self._start_ready = self._stop_ready = False
            return False
        if start <= self.release:
            self._start_ready = True
        if stop <= self.release:
            self._stop_ready = True
        start_edge = start >= self.threshold and self._start_ready
        stop_edge = stop >= self.threshold and self._stop_ready
        if start_edge:
            self._start_ready = False
        if stop_edge:
            self._stop_ready = False
        if stop_edge:
            self.active = False
        elif start_edge:
            self.active = True
        return self.active


class ToggleFollow:
    def __init__(self, threshold=.55, release=.1):
        self.threshold, self.release = threshold, release
        self.ready = self.active = False

    def update(self, value, valid=True):
        if not valid:
            self.ready = self.active = False
        elif value <= self.release:
            self.ready = True
        elif value >= self.threshold and self.ready:
            self.active = not self.active
            self.ready = False
        return self.active
