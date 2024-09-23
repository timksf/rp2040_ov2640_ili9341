with open("memory.bin", "wb") as f: f.write(bytes.fromhex("".join(line[17:] for line in open("memory.txt"))))
