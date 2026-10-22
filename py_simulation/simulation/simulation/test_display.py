#!/usr/bin/env python3
"""Quick test to verify matplotlib figure display works."""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

# Test 1: Simple plot
print("Creating test figure...")
fig, ax = plt.subplots(figsize=(8, 6))
x = np.linspace(0, 10, 100)
y = np.sin(x)
ax.plot(x, y, 'b-', label='sin(x)')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Test Figure - Close this window to continue')
ax.legend()
ax.grid(True)

print("Showing figure (close window to continue)...")
plt.show()
print("Figure closed, test complete!")
