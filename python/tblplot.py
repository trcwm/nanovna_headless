#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = np.array([
10533,
27246,
32698,
24636,
6393,
-14493,
-29389,
-32138,
-21605,
-2143,
18205,
31029,
31029,
18205,
-2143,
-21605,
-32138,
-29389,
-14493,
6393,
24636,
32698,
27246,
10533,
-10533,
-27246,
-32698,
-24636,
-6393,
14493,
29389,
32138,
21605,
2143,
-18205,
-31029,
-31029,
-18205,
2143,
21605,
32138,
29389,
14493,
-6393,
-24636,
-32698,
-27246,
-10533])

data2 = np.array([
31029,
18205,
-2143,
-21605,
-32138,
-29389,
-14493,
6393,
24636,
32698,
27246,
10533,
-10533,
-27246,
-32698,
-24636,
-6393,
14493,
29389,
32138,
21605,
2143,
-18205,
-31029,
-31029,
-18205,
2143,
21605,
32138,
29389,
14493,
-6393,
-24636,
-32698,
-27246,
-10533,
10533,
27246,
32698,
24636,
6393,
-14493,
-29389,
-32138,
-21605,
-2143,
18205,
31029])

n = np.linspace(0, len(data), len(data))

plt.figure(1)
plt.plot(n, data, n, 32000.0*np.sin(2.0*3.1415927*(n+0.5) * 5000/48000))
plt.grid()
plt.show()