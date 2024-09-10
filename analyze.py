import pathlib
import numpy as np

def human_readable_size(size_in_bytes):
    # Define units
    units = ['B', 'KB', 'MB', 'GB', 'TB', 'PB']
    # Find the right unit
    index = 0
    size = float(size_in_bytes)
    
    while size >= 1024 and index < len(units) - 1:
        size /= 1024
        index += 1
    
    # Return formatted string with 2 decimal places
    return f"{size:.2f} {units[index]}"

data_bytes = np.fromfile("eee.bin", dtype=np.uint8)

#find SOI and EOI markers
start_idcs = np.where((data_bytes[:-1] == 0xFF) & (data_bytes[1:] == 0xD8))[0]
end_idcs = np.where((data_bytes[:-1] == 0xFF) & (data_bytes[1:] == 0xD9))[0]

im_idcs = np.array(list(zip(start_idcs, end_idcs+2)))
sizes = im_idcs[..., 1] - im_idcs[..., 0]


for file in pathlib.Path("output").iterdir():
    if file.is_file() and file.name.startswith("file"):
        file.unlink()

prefix = "output/file"
for i, (start, size) in enumerate(zip(start_idcs, sizes)):
    print(f"Image {i}, size: {human_readable_size(size)}")
    jpeg_bytes = data_bytes[start:start+size]
    filename = f"{prefix}_{i+1}.jpg"
    with open(filename, 'wb') as file:
        file.write(jpeg_bytes)