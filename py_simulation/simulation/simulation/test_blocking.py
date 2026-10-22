#!/usr/bin/env python3
"""Test script to verify blocking plt.show() behavior."""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

print("=" * 70)
print("TEST: Figure Blocking Behavior")
print("=" * 70)

# Test 1
print("\nTest 1: Creating first figure...")
fig1, ax1 = plt.subplots(figsize=(8, 6))
x = np.linspace(0, 10, 100)
ax1.plot(x, np.sin(x), 'b-', label='sin(x)', linewidth=2)
ax1.set_title('Figure 1: sin(x)')
ax1.legend()
ax1.grid(True)

# Test 2
print("Test 2: Creating second figure...")
fig2, ax2 = plt.subplots(figsize=(8, 6))
ax2.plot(x, np.cos(x), 'r-', label='cos(x)', linewidth=2)
ax2.set_title('Figure 2: cos(x)')
ax2.legend()
ax2.grid(True)

print("\n>>> Close ALL figure windows to continue <<<\n")
plt.show(block=True)

print("All windows closed! Moving to next section...\n")

# Test 3
print("Test 3: Creating third figure...")
fig3, ax3 = plt.subplots(figsize=(8, 6))
ax3.plot(x, np.tan(x), 'g-', label='tan(x)', linewidth=2)
ax3.set_ylim(-10, 10)
ax3.set_title('Figure 3: tan(x)')
ax3.legend()
ax3.grid(True)

print("\n>>> Close ALL figure windows to continue <<<\n")
plt.show(block=True)

print("Test completed successfully!")
