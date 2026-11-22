"""Mission state machine with latching abort and operator-gated restart."""


class MissionFSM:
    states = ("IDLE", "RUNNING", "HOLDING", "ABORTING", "LANDING", "DONE", "FAULT")

    def __init__(self, route_length):
        if route_length < 1:
            raise ValueError("route_length must be positive")
        self.route_length = route_length
        self.state = "IDLE"
        self.target_index = 0
        self.hold_elapsed = 0.0
        self.hold_required = 2.0
        self.abort_reason = None

    def start_route(self):
        if self.state != "IDLE":
            raise RuntimeError("mission cannot start from state " + self.state)
        self.state = "RUNNING"
        self.target_index = 0

    def report_reached(self, index):
        if self.state != "RUNNING" or index != self.target_index:
            raise RuntimeError("reached report out of sequence")
        self.state = "HOLDING"
        self.hold_elapsed = 0.0

    def ack_hold(self):
        if self.state != "HOLDING" or self.hold_elapsed < self.hold_required:
            raise RuntimeError("hold not satisfied")
        if self.target_index + 1 < self.route_length:
            self.target_index += 1
            self.state = "RUNNING"
        else:
            self.state = "LANDING"

    def report_landed(self):
        if self.state != "LANDING":
            raise RuntimeError("landed report outside LANDING")
        self.state = "DONE"

    def abort(self, reason):
        self.abort_reason = reason
        if self.state not in ("DONE", "FAULT"):
            self.state = "ABORTING"

    def can_resume_without_operator_reset(self):
        return False
