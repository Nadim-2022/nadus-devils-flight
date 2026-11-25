# Guided bench tests for the MTF-01P
# each test says what to do, collects a few seconds, prints one short summary
#
# python3 mtf01p_test.py --test height
# python3 mtf01p_test.py --test covered
# python3 mtf01p_test.py --test floor
# python3 mtf01p_test.py --test direction

import time


COLLECT_SECONDS = 3.0
SETTLE_SECONDS = 1.0 # hand still moving just after Enter
MAX_HEIGHT_ERROR_CM = 4.0 # or 3 % of the height, whichever is bigger


def wait_for_enter(text):
    print()
    print(text)
    input("Press Enter when ready... ")


# readings from the next few seconds, old bytes thrown away first
def collect(port, reader, seconds=COLLECT_SECONDS):
    port.reset_input_buffer()
    readings = []
    start = time.time()
    print("Collecting for %.0f s..." % seconds)
    while time.time() - start < seconds:
        for rx_byte in port.read(256):
            reading = reader.add_byte(rx_byte)
            if reading is not None:
                readings.append(reading)
    if len(readings) == 0:
        print("No frames! Check the wiring and that the sensor is set to Micolink.")
    return readings


def average(values):
    if len(values) == 0:
        return 0.0
    return sum(values) / len(values)


def percent(readings, key):
    if len(readings) == 0:
        return 0.0
    return 100.0 * sum(1 for reading in readings if reading[key]) / len(readings)


def values_seen(readings, key):
    return sorted(set(reading[key] for reading in readings))


# ground distance moved along x / y, flow * distance * time
def distance_moved(readings):
    moved_x = 0.0
    moved_y = 0.0
    for previous, reading in zip(readings, readings[1:]):
        time_step = (reading["sensor_time_ms"] - previous["sensor_time_ms"]) * 0.001
        if time_step <= 0.0 or time_step > 0.1:
            time_step = 0.01
        moved_x += reading["speed_x"] * time_step
        moved_y += reading["speed_y"] * time_step
    return moved_x, moved_y


#--------------------Height-------------------------//
def height_test(port, reader):
    print("=== HEIGHT TEST ===")
    print("Sensor pointing straight down at the floor, measure from the lens.")
    results = []
    for target_cm in (30, 100, 200):
        wait_for_enter("Hold the sensor still at %d cm above the floor (tape measure)." % target_cm)
        time.sleep(SETTLE_SECONDS)
        readings = collect(port, reader)
        if len(readings) == 0:
            continue
        distances = [reading["distance_mm"] for reading in readings if reading["distance_mm"] > 0]
        results.append((target_cm, readings, distances))

    print()
    print("=== HEIGHT RESULT ===")
    print("target    average     min     max   error   ok %   status seen   strength  result")
    for target_cm, readings, distances in results:
        if len(distances) == 0:
            print("%4d cm   no distance readings" % target_cm)
            continue
        average_mm = average(distances)
        error_cm = average_mm / 10.0 - target_cm
        allowed_cm = max(MAX_HEIGHT_ERROR_CM, 0.03 * target_cm)
        ok_percent = percent(readings, "distance_ok")
        if abs(error_cm) <= allowed_cm and ok_percent >= 100.0:
            result = "GOOD"
        else:
            result = "CHECK (more than %.0f cm off)" % allowed_cm
        print("%4d cm   %5.0f mm %5d mm %5d mm %+4.0f cm %5.0f   %-12s  %5.0f    %s"
              % (target_cm, average_mm, min(distances), max(distances),
                 error_cm, ok_percent,
                 values_seen(readings, "distance_status"),
                 average([reading["strength"] for reading in readings]), result))
    print("A big min / max spread means it moved while collecting.")


#--------------------Covered-------------------------//
def summary_line(name, readings):
    if len(readings) == 0:
        print("%-9s no frames" % name)
        return
    qualities = [reading["flow_quality"] for reading in readings]
    print("%-9s dist %5.0f mm  dist status %-10s dist ok %3.0f %% | quality %3.0f (%3d..%3d)  flow status %-10s flow ok %3.0f %%"
          % (name, average([reading["distance_mm"] for reading in readings]),
             values_seen(readings, "distance_status"), percent(readings, "distance_ok"),
             average(qualities), min(qualities), max(qualities),
             values_seen(readings, "flow_status"), percent(readings, "flow_ok")))


def covered_test(port, reader):
    print("=== COVERED TEST ===")
    wait_for_enter("Uncovered: hold the sensor still about 1 m over a patterned floor.")
    open_readings = collect(port, reader)
    wait_for_enter("Covered: put your finger or tape over BOTH lenses.")
    covered_readings = collect(port, reader)

    print()
    print("=== COVERED RESULT ===")
    summary_line("uncovered", open_readings)
    summary_line("covered", covered_readings)
    print("The status values in the uncovered line are the good ones (the code guesses 1).")
    print("Covered should give a different status, 0 distance or very low quality.")


#--------------------Floor-------------------------//
def floor_test(port, reader):
    print("=== FLOOR TEST ===")
    print("Hold the sensor still about 1 m over different floors, one at a time.")
    results = []
    while True:
        print()
        name = input("Name of the floor (empty to finish): ").strip()
        if name == "":
            break
        wait_for_enter("Hold it still about 1 m over: %s" % name)
        readings = collect(port, reader)
        results.append((name, readings))

    print()
    print("=== FLOOR RESULT ===")
    print("floor                quality  min  max  flow ok %  still speed avg / max  flow status seen")
    for name, readings in results:
        if len(readings) == 0:
            print("%-20s no frames" % name)
            continue
        qualities = [reading["flow_quality"] for reading in readings]
        # held still, so any speed here is noise
        speeds = [(reading["speed_x"] ** 2 + reading["speed_y"] ** 2) ** 0.5 for reading in readings]
        print("%-20s %7.0f  %3d  %3d  %9.0f  %6.2f / %4.2f m/s     %s"
              % (name, average(qualities), min(qualities), max(qualities),
                 percent(readings, "flow_ok"), average(speeds), max(speeds),
                 values_seen(readings, "flow_status")))
    print("Held still, so the speed should be near 0. A floor with big still speed is no good for holding.")
    print("FLOW_MIN_QUALITY in main.cpp is 50: pick it between the bad and the good floors.")


#--------------------Direction-------------------------//
def direction_test(port, reader):
    print("=== DIRECTION TEST ===")
    print("Sensor pointing down, mounted the way it will sit on the drone,")
    print("so the drone's front is your front. Keep it level, don't tilt it.")

    wait_for_enter("Hold it at about 50 cm. After Enter, slide it FORWARD about 30 cm, slowly over 2 s.")
    forward_x, forward_y = distance_moved(collect(port, reader))

    wait_for_enter("Back to the start. After Enter, slide it to the RIGHT about 30 cm, slowly over 2 s.")
    right_x, right_y = distance_moved(collect(port, reader))

    print()
    print("=== DIRECTION RESULT ===")
    print("forward slide: x %+.2f m  y %+.2f m" % (forward_x, forward_y))
    print("right slide:   x %+.2f m  y %+.2f m" % (right_x, right_y))

    forward_on_y = abs(forward_y) > abs(forward_x)
    right_on_y = abs(right_y) > abs(right_x)
    if forward_on_y == right_on_y:
        print("Both slides moved the same axis, do it again more carefully.")
        return

    forward_value = forward_y if forward_on_y else forward_x
    right_value = right_y if right_on_y else right_x
    if abs(forward_value) < 0.05 or abs(right_value) < 0.05:
        print("Moved less than 5 cm, slide further or check the height.")
        return

    print()
    print("Settings for main.cpp:")
    print("static constexpr bool FLOW_SWAP_AXES = %s;" % ("true" if forward_on_y else "false"))
    print("static constexpr float FLOW_FORWARD_DIRECTION = %s1.0f;" % ("+" if forward_value > 0 else "-"))
    print("static constexpr float FLOW_RIGHT_DIRECTION = %s1.0f;" % ("+" if right_value > 0 else "-"))
    print("(measured about 30 cm? Moved distance should be roughly that.)")


#--------------------Right-------------------------//
# forward is known (sensor Y, minus), this only finds the sign of X
def right_test(port, reader):
    print("=== RIGHT TEST ===")
    print("Drone on the floor, nose forward. Push it to the RIGHT along the floor,")
    print("about a forearm length. No need to measure, only the sign counts.")
    signs = []
    for attempt in range(1, 4):
        wait_for_enter("Try %d of 3: after Enter, push it to the right." % attempt)
        moved_x, moved_y = distance_moved(collect(port, reader))
        print("X moved %+.2f m (Y %+.2f m)" % (moved_x, moved_y))
        if abs(moved_x) >= 0.05:
            signs.append(1 if moved_x > 0 else -1)

    print()
    print("=== RIGHT RESULT ===")
    if len(signs) >= 2 and all(sign == signs[0] for sign in signs):
        print("static constexpr float FLOW_RIGHT_DIRECTION = %s1.0f;" % ("+" if signs[0] > 0 else "-"))
    else:
        print("Tries didn't agree, push a bit further and straighter and run it again.")


#--------------------Axes-------------------------//
# 3 pushes one way, which sensor axis moved most and its sign
# returns ("x" or "y", +1 or -1), or None when the pushes don't agree
def pushes(port, reader, direction_name):
    results = []
    for attempt in range(1, 4):
        wait_for_enter("%s %d of 3: after Enter, push the drone %s along the floor, about a hand's width."
                       % (direction_name.capitalize(), attempt, direction_name.upper()))
        moved_x, moved_y = distance_moved(collect(port, reader))
        print("X moved %+.2f m, Y moved %+.2f m" % (moved_x, moved_y))
        if max(abs(moved_x), abs(moved_y)) < 0.03:
            print("Too small, push a bit further next time.")
            continue
        if abs(moved_x) > abs(moved_y):
            results.append(("x", 1 if moved_x > 0 else -1))
        else:
            results.append(("y", 1 if moved_y > 0 else -1))

    for result in results:
        if results.count(result) >= 2:
            return result
    return None


def axes_test(port, reader):
    print("=== AXES TEST ===")
    print("Drone on the floor, nose pointing away from you.")
    print("Push it straight, no need to measure, only the direction counts.")

    forward = pushes(port, reader, "forward")
    right = pushes(port, reader, "right")

    print()
    print("=== AXES RESULT ===")
    if forward is None or right is None:
        print("Pushes didn't agree, push straighter and run it again.")
        return
    if forward[0] == right[0]:
        print("Forward and right both moved sensor %s, run it again and push straighter." % forward[0].upper())
        return

    print("Forward = sensor %s %s, right = sensor %s %s"
          % (forward[0].upper(), "+" if forward[1] > 0 else "-", right[0].upper(), "+" if right[1] > 0 else "-"))
    print()
    print("Settings for main.cpp:")
    print("static constexpr bool FLOW_SWAP_AXES = %s;" % ("true" if forward[0] == "y" else "false"))
    print("static constexpr float FLOW_FORWARD_DIRECTION = %s1.0f;" % ("+" if forward[1] > 0 else "-"))
    print("static constexpr float FLOW_RIGHT_DIRECTION = %s1.0f;" % ("+" if right[1] > 0 else "-"))


#--------------------Scale-------------------------//
# push a known distance along a tape measure, compare with what the flow says
def scale_test(port, reader):
    print("=== SCALE TEST ===")
    print("Drone on the floor next to a tape measure, nose forward.")
    text = input("How far will you push it forward, in cm (e.g. 30): ").strip()
    try:
        real_m = float(text) / 100.0
    except ValueError:
        print("Not a number")
        return

    ratios = []
    for attempt in range(1, 4):
        wait_for_enter("Push %d of 3: after Enter, push it FORWARD exactly %.0f cm along the tape, then stop."
                       % (attempt, real_m * 100.0))
        readings = collect(port, reader, 4.0)
        moved_x, moved_y = distance_moved(readings)
        moved = (moved_x ** 2 + moved_y ** 2) ** 0.5
        height = average([reading["distance_m"] for reading in readings])
        print("flow says %.2f m (real %.2f m) at %.2f m height" % (moved, real_m, height))
        if moved > 0.01:
            ratios.append(moved / real_m)

    print()
    print("=== SCALE RESULT ===")
    if len(ratios) == 0:
        print("No movement seen")
        return
    ratio = average(ratios)
    print("flow / real = %.2f (1.00 = scaling right)" % ratio)
    if abs(ratio - 1.0) > 0.25:
        print("Off by more than 25 %%, the flow needs a scale: FLOW_SCALE = %.2ff" % (1.0 / ratio))


TESTS = {
    "scale": scale_test,
    "axes": axes_test,
    "right": right_test,
    "height": height_test,
    "covered": covered_test,
    "floor": floor_test,
    "direction": direction_test,
}
