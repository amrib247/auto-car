def controls_from_pressed_keys(pressed_keys: set[str]) -> tuple[float, float]:
    steering = float("d" in pressed_keys) - float("a" in pressed_keys)
    throttle = float("w" in pressed_keys) - float("s" in pressed_keys)
    return steering, throttle