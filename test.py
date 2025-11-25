data = "af0f6d5813000000"
def get_bits(bitstring, start, length):
    return int(bitstring[start:start + length], 2)

def decode_frc_payload(payload):
    bits = ''.join(f'{b:08b}' for b in payload)
    return {
        "match_time":      get_bits(bits, 56, 8),
        "match_number":    get_bits(bits, 46, 10),
        "replay_number":   get_bits(bits, 40, 6),
        "red_alliance":    get_bits(bits, 39, 1),
        "enabled":         get_bits(bits, 38, 1),
        "autonomous":      get_bits(bits, 37, 1),
        "test":            get_bits(bits, 36, 1),
        "watchdog":        get_bits(bits, 35, 1),
        "tournament_type": get_bits(bits, 32, 3),
        "year":            2000 + get_bits(bits, 26, 6) - 36,
        "month":           get_bits(bits, 22, 4) + 1,
        "day":             get_bits(bits, 17, 5),
        "seconds":         get_bits(bits, 11, 6),
        "minutes":         get_bits(bits, 5, 6),
        "hours":           get_bits(bits, 0, 5),
    }

print(decode_frc_payload(bytes.fromhex(data)))