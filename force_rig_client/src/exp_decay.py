import itertools


def sequence(decay: float):
    if not decay < 1:
        raise ValueError(f"Bad decay factor: {decay}")
    out = []
    for i in itertools.count():
        b = decay**i
        if round(b * 100) < 1:
            break
        s = -1 if i % 2 == 0 else +1
        out.append(s * b)
    return out


seq = sequence(0.9)
print(f"length: {len(seq)}")
print("".join(f"{round(x * 100):+d}," for x in seq))
