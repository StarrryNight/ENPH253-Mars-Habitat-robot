from statemachine import State, StateMachine


class RobotStateMachine(StateMachine):
    line_following = State(initial=True)
    teletubbying = State()

    class metal_detecting(State.Compound):
        scanning = State(initial=True)
        retrieving = State()
        _cycle = scanning.to(retrieving, cond="metal_detected")

    class habitating(State.Compound):
        grabbing = State(initial=True)
        holding = State()
        placing = State()
        _cycle = grabbing.to(holding) | holding.to(placing)

    cycle = (
        line_following.to(metal_detecting, cond="enter_metal_detection")
        | metal_detecting.scanning.to(line_following, cond="metal_not_detected")
        | metal_detecting.retrieving.to(line_following)
        | teletubbying.to(line_following)
        | line_following.to(teletubbying, cond="enter_teletubby")
        | line_following.to(habitating, cond="enter_habitat")
        | habitating.to(line_following)
    )

    # Condition flags — set by mrs_krabs before calling send("cycle")
    _metal_detected: bool = False
    _enter_metal_detection: bool = False
    _metal_not_detected: bool = False
    _enter_teletubby: bool = False
    _enter_habitat: bool = False

    def metal_detected(self) -> bool:
        return self._metal_detected

    def metal_not_detected(self) -> bool:
        return self._metal_not_detected

    def enter_metal_detection(self) -> bool:
        return self._enter_metal_detection

    def enter_teletubby(self) -> bool:
        return self._enter_teletubby

    def enter_habitat(self) -> bool:
        return self._enter_habitat

    def current_top_state_id(self) -> str:
        """Returns the top-level state name the ESP32 understands.
        Compound sub-states (scanning, retrieving, etc.) are mapped to their parent."""
        _parent_map = {
            "scanning":   "metal_detecting",
            "retrieving": "metal_detecting",
            "grabbing":   "habitating",
            "holding":    "habitating",
            "placing":    "habitating",
        }
        return _parent_map.get(self.current_state.id, self.current_state.id)
