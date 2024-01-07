import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

columns = ["x", "y"]
df = pd.read_csv("data2.csv", usecols=columns)

x_values = np.array(df["x"])
y_values = np.array(df["y"])

plt.plot(x_values, y_values)
plt.show()

