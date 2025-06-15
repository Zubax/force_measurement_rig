# Your input list
values = [
    -100,
    -90,
    -81,
    +73,
    +66,
    -59,
    -53,
    +48,
    +43,
    -39,
    -35,
    +31,
    +28,
    -25,
    -23,
    +21,
    +19,
    -17,
    -15,
    +14,
    -12,
    +11,
    -10,
    +9,
    -8,
    +7,
    -6,
    -6,
    -5,
    +5,
    -4,
    +4,
    -3,
    +3,
    -3,
    +3,
    -2,
    +2,
    -2,
    +2,
    -1,
    +1,
    -1,
    +1,
    -1,
    +1,
    -1,
    +1,
    -1,
    +1,
    -1,
]


def add_offset(values: list, offset: int) -> list:
    new_values = []
    for x in values:
        if x + offset == 0:
            new_values.append(x)
        elif abs(x + offset) > 100:
            new_values.append(x)
        else:
            new_values.append(x + offset)
    return new_values


# Set the constant you want to add
constant = 0

# Apply the function
new_values = add_offset(values, constant)

# Print the result
print(f"offset: {constant}")
print(f"average: {sum(new_values) / len(new_values)}")
print(len(new_values))
print(new_values)
