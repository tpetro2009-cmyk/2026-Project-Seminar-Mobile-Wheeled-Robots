import csv
from pathlib import Path

import matplotlib.pyplot as plt


RESULTS = Path(__file__).parent / 'results'

EXPERIMENTS = {
    1: 'experiment_1_global_small.csv',
    2: 'experiment_2_global_large.csv',
    3: 'experiment_3_small_error_small_particles.csv',
    4: 'experiment_4_small_error_large_particles.csv',
    5: 'experiment_5_large_error_small_particles.csv',
    6: 'experiment_6_large_error_large_particles.csv',
}

POSITION_THRESHOLD = 0.15
YAW_THRESHOLD = 0.15
CONVERGENCE_TIMES = {
    1: 6.56,
    2: 9.12,
    3: 17.86,
    4: 6.56,
    5: None,
    6: None,
}

def load_csv(path):
    with open(path, newline='') as file:
        return list(csv.DictReader(file))


def plot_experiment(number, filename):
    rows = load_csv(RESULTS / filename)

    time = [float(row['time']) for row in rows]
    gt_x = [float(row['gt_x']) for row in rows]
    gt_y = [float(row['gt_y']) for row in rows]
    amcl_x = [float(row['amcl_x']) for row in rows]
    amcl_y = [float(row['amcl_y']) for row in rows]
    position_error = [float(row['position_error']) for row in rows]
    yaw_error = [float(row['yaw_error']) for row in rows]

    convergence_time = CONVERGENCE_TIMES[number]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # 1. Траектории
    axes[0].plot(gt_x, gt_y, label='Ground Truth')
    axes[0].plot(amcl_x, amcl_y, label='AMCL')
    axes[0].set_title('Trajectories')
    axes[0].set_xlabel('X, m')
    axes[0].set_ylabel('Y, m')
    axes[0].axis('equal')
    axes[0].grid(True)
    axes[0].legend()

    # 2. Ошибка положения
    axes[1].plot(time, position_error)
    axes[1].axhline(
        POSITION_THRESHOLD,
        linestyle='--',
        label='Threshold = 0.15 m'
    )

    if convergence_time is not None:
        axes[1].axvline(
            convergence_time,
            color='red',
            linestyle=':',
            label=f'Convergence = {convergence_time} s'
        )

    axes[1].set_title('Position error')
    axes[1].set_xlabel('Time, s')
    axes[1].set_ylabel('Error, m')
    axes[1].grid(True)
    axes[1].legend()

    # 3. Ошибка ориентации
    axes[2].plot(time, yaw_error)
    axes[2].axhline(
        YAW_THRESHOLD,
        linestyle='--',
        label='Threshold = 0.15 rad'
    )

    if convergence_time is not None:
        axes[2].axvline(
            convergence_time,
            color='red',
            linestyle=':',
            label=f'Convergence = {convergence_time} s'
        )

    axes[2].set_title('Yaw error')
    axes[2].set_xlabel('Time, s')
    axes[2].set_ylabel('Error, rad')
    axes[2].grid(True)
    axes[2].legend()

    fig.suptitle(f'Experiment {number}')
    fig.tight_layout()

    output = RESULTS / f'experiment_{number}.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)

    print(f'Saved: {output}')

   


def main():
    for number, filename in EXPERIMENTS.items():
        plot_experiment(number, filename)


if __name__ == '__main__':
    main()
