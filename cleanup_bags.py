import shutil
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--dir", default="bags")
args = parser.parse_args()
dir = args.dir


# find recursively all the directories that contain an .mcap file
dirs_to_clean = list(
    filter(
        lambda x: set(x[2]) == {"metadata.yaml", f"{x[0].split('/')[-1]}_0.mcap"},
        os.walk(dir),
    )
)
print(
    f"Found {len(dirs_to_clean)} directories to clean:",
    "".join(list("\n\t - " + x[0] for x in dirs_to_clean)),
)

# for each, move the mcap file in the parent directory, removes the _0 from the name, and removes the directory
for dirpath, _, filenames in dirs_to_clean:
    basename = dirpath.split("/")[-1]
    shutil.move(
        os.path.join(dirpath, f"{basename}_0.mcap"),
        os.path.join(dirpath, "..", f"{basename}.mcap"),
    )
    shutil.rmtree(dirpath)
