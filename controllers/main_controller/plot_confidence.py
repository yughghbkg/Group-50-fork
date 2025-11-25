import matplotlib.pyplot as plt
import csv

steps = []
confidence = []

with open("confidence_history.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        steps.append(int(row["step"]))
        confidence.append(float(row["confidence"]))

plt.figure(figsize=(10,5))
plt.plot(steps, confidence)
plt.title("Lost Detection Confidence Over Time")
plt.xlabel("Time Step")
plt.ylabel("Confidence")
plt.grid(True)
plt.tight_layout()
plt.savefig("confidence_plot.png")

print("Saved confidence_plot.png")
