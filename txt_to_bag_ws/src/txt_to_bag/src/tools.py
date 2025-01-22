import numpy as np
import os

def convert_ultra_data():
  ultra_path = "/data/05_dataset/00_slam/04_relocation/P3C_0528/0528/log/ultra.log"
  triple_ultra_path = "/data/05_dataset/00_slam/04_relocation/P3C_0528/0528/log/triple_ultra.log"
  left_front_ultra_path = "/data/05_dataset/00_slam/04_relocation/P3C_0528/0528/log/left_front.log"
  left_back_ultra_path = "/data/05_dataset/00_slam/04_relocation/P3C_0528/0528/log/left_back.log"

  triple_ultra_file = open(triple_ultra_path, 'w')
  left_front_ultra_file = open(left_front_ultra_path, 'w')
  left_back_ultra_file = open(left_back_ultra_path, 'w')

  with open(ultra_path, 'r') as f:
    ultra_lines = f.readlines()
    for ultra_line in ultra_lines:
      ultra_line = ultra_line.strip().split(" ")
      id = int(ultra_line[-1])
      if id == 0:
        ultra_data = ultra_line[:4]
        ultra_data.append("0")
        ultra_data = ' '.join(ultra_data) + "\n"
        triple_ultra_file.write(ultra_data)
      elif id == 1:
        ultra_data = ultra_line[0:1] + ultra_line[4:5]
        ultra_data.append("0")
        ultra_data = ' '.join(ultra_data) + "\n"
        left_front_ultra_file.write(ultra_data)
      elif id == 2:
        ultra_data = ultra_line[0:1] + ultra_line[5:6]
        ultra_data.append("0")
        ultra_data = ' '.join(ultra_data) + "\n"
        left_back_ultra_file.write(ultra_data)

  triple_ultra_file.close()
  left_front_ultra_file.close()
  left_back_ultra_file.close()


if __name__ == "__main__":
  convert_ultra_data()