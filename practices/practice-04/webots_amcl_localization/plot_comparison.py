import csv
from pathlib import Path
import matplotlib.pyplot as plt

results = Path(__file__).parent / 'results'

with open(results / 'comparison.csv', newline='') as file:
    rows = list(csv.DictReader(file))

names = [f"Experiment {row['Experiment']}" for row in rows]
times = [
    float(row['Convergence time (s)'])
    if row['Convergence time (s)'] else 0
    for row in rows
]

fig, ax = plt.subplots(figsize=(10, 5))

bars = ax.bar(names, times)

for bar, row in zip(bars, rows):
    if row['Status'] == 'Not converged':
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            0.5,
            'Not converged',
            ha='center',
            rotation=90
        )
    else:
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.3,
            f"{bar.get_height():.2f} s",
            ha='center'
        )

ax.set_title('AMCL Localization Convergence Time')
ax.set_ylabel('Convergence time, s')
ax.set_ylim(0, max(times) + 5)
ax.grid(axis='y')
plt.xticks(rotation=15)
plt.tight_layout()

output = results / 'comparison.png'
plt.savefig(output, dpi=150)
plt.close()

print(f'Saved: {output}')
