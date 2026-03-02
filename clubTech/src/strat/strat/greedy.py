import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from aco import uniqueStacksPos, uniqueAtticsPos, stacksPos, atticsPos, kGrabBoxPositionAngle, kAtticsPosition


def invertDict(
    l,
    f
):
    by_value = defaultdict(set)

    for e in l:
        by_value[f(e)].add(e)

    return {f(e): by_value[f(e)] for e in l}

invStacksPartitionMap = invertDict(range(len(kGrabBoxPositionAngle)), lambda i: uniqueStacksPos.index(stacksPos[i]))
invAtticsPartitionMap = invertDict(range(len(kAtticsPosition)), lambda i: uniqueAtticsPos.index(atticsPos[i]))

class greedy:
    def __init__(self, current_pos, kaplas_pos, garden_pos, cap=1):
        self.kaplas_orientation = [o for _, o in kaplas_pos]
        self.kaplas = [np.array(k) for k, _ in kaplas_pos]
        self.garden_orientation = [o for _, o in garden_pos]
        self.garden_pos = [np.array(g) for g, _ in garden_pos]
        self.init_pos = np.array(current_pos)
        self.cap = cap
        self.capacity = [False]*cap

    def compute_min_point(self, curr, idx, kap: bool):
        inv = invStacksPartitionMap if kap else invAtticsPartitionMap
        pos = self.kaplas if kap else self.garden_pos
        positions = [(i, pos[i], np.linalg.norm(curr - pos[i])) for i in inv[idx]] 
        return min(positions, key=lambda x: x[2])
    
    def solution(self):
        N = len(uniqueStacksPos)
        k_indexes = list(range(N))
        g_indexes = list(range(len(uniqueAtticsPos)))
        current_pos = self.init_pos
        solution = []
        for i in range(2*N):
            if not g_indexes:
                break
            grab = False
            if not any(self.capacity):
                grab = True
            else:
                g_idx, (g_indx_real, g_pos, g_min) = min(enumerate(self.compute_min_point(current_pos, x, False) for x in g_indexes), key=lambda x: x[1][2])
            
            if not all(self.capacity) or grab:
                k_idx, (k_idx_real, k_pos, k_min) = min(enumerate(self.compute_min_point(current_pos, x, True) for x in k_indexes), key=lambda x: x[1][2])
                if grab or k_min == min(k_min, g_min):
                    solution.append((tuple(k_pos), self.kaplas_orientation[k_idx_real]))
                    k_indexes.pop(k_idx)
                    current_pos = k_pos
                    self.capacity[i % self.cap] = True
                    continue

            solution.append((tuple(g_pos), self.garden_orientation[g_indx_real]))
            g_indexes.pop(g_idx)
            current_pos = g_pos
            self.capacity[i % self.cap] = False
                
        return solution


def plot_all_and_path(path, kaplas_pos, garden_pos, start_pos=None, title="Greedy"):
    # extraire positions
    kap_xy = np.array([p for p in kaplas_pos], dtype=float)
    gar_xy = np.array([p for p in garden_pos], dtype=float)
    path_xy = np.array([pos for (pos, _) in path], dtype=float)

    plt.figure(figsize=(11, 6))

    # Kaplas (grab)
    plt.scatter(
        kap_xy[:, 0], kap_xy[:, 1],
        label="Kaplas (grab)",
        color="tab:blue",
        s=60
    )

    # Attics / Garden (étoiles)
    plt.scatter(
        gar_xy[:, 0], gar_xy[:, 1],
        label="Attics (drop)",
        marker="*",
        color="tab:orange",
        s=180,
        edgecolors="black",
        linewidths=1
    )

    # Start
    if start_pos is not None:
        sx, sy = start_pos
        plt.scatter(
            [sx], [sy],
            marker="*",
            s=350,
            color="red",
            edgecolors="black",
            label="Start"
        )

    # Chemin greedy
    plt.plot(
        path_xy[:, 0], path_xy[:, 1],
        color="black",
        linewidth=2,
        marker="o",
        label="Greedy path"
    )

    # Optionnel : numéroter l’ordre de visite
    for i, (x, y) in enumerate(path_xy):
        plt.text(x, y, str(i), fontsize=9, color="black")

    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("greedy_plot.png", dpi=200, bbox_inches="tight")
    print("Saved greedy_plot.png")


if __name__ == "__main__":
    start = uniqueAtticsPos[0]     
    kaplas_pos = kGrabBoxPositionAngle
    garden_pos = kAtticsPosition

    g = greedy(start, kaplas_pos, garden_pos, cap=1)
    sol = g.solution()                     

    for i, (pos, ang) in enumerate(sol):
        print(f"{i:02d}: pos={pos} angle={ang:.3f}")

    plot_all_and_path(
        sol,
        uniqueStacksPos,
        uniqueAtticsPos,
        start_pos=start,
        title="Greedy solution"
    )     
